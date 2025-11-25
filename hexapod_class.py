import random
from weakref import ref
import sys
import time
import os
import dill as pickle
import numpy as np

# Custom Modules
from real_time_graph_class import RealTimeGraph
import control_system_class as cs
from kalman_filter import Kalman
import morphology_class as mc

sys.path.append('/home/pi/SpiderPi/')  # Access the codes from parent directory
import HiwonderSDK.Board as Board


# WARNING!!!
# READING DATA FROM ANY SERVO RIGHT AFTER ANY HAS BEEN COMMANDED TO MOVE STOPS THE SERVO FROM MOVING.
# Introduce a time window to let servo "recover". It takes about servo speed/1000 seconds to reach desired angle if not loaded.
# READING DATA FROM MULTIPLE JOINTS AT THE SAME TIME CRASHES.

class Joint:
    """
    Class that represents joints in the legs.
    This class offers tools to control, diagnose and manipulate each servo individually.
    """

    # Physical Parameters of the servos - Do not change
    min_position = 0  # -120 deg
    mid_position = 500  # 0 deg
    max_position = 1000  # +120 deg

    def __init__(self, parent, servo_id, joint_type):
        
        # Servo ID parameters
        self.parent = ref(parent)  # Joints parent is the corresponding leg
        self.hexapod = self.parent().parent  # Create a reference to the hexapod to access its variables
        self.servo_id = servo_id  # int from 1 to 18 unique per servo
        self.type = joint_type  # Shoulder/Knee/Paw
        self.name = f'{parent.name} {joint_type}'  # Example: "RF Shoulder" (Right Front Shoulder)
        self.side = parent.name[0]  # Its either R or L depending on the leg name. Used to separate left leg from right leg joints
        self.platform = sys.platform  # Check if code ran on PC or RBpi ... for debugging purposes
        
        # Servo control parameters at first inherited from hexapod, but possible to tweak subjectively
        self.theta = self.hexapod().theta  # Integer (0<theta<~150). How far the shoulder moves during Swing/Stance.
        self.speed = self.hexapod().speed  # How long it takes in [ms] for servo to reach the desired position.
        self.reading_window = self.hexapod().reading_window  # Float (0.1<x<0.7). During this time, servo can't move.
        self.writing_window_factor = self.hexapod().writing_recover_factor  # No need to change... Time for servo to "recover" after moving command.
        self.up_v_threshold = self.hexapod().up_v_threshold  # Voltage threshold in [mV]. If reached "LO" -> "SW". Higher values -> Lifting leg higher -> body drops.
        self.down_v_threshold = self.hexapod().down_v_threshold  # Voltage threshold in [mV]. If reached "TD" -> "ST". Higher values -> Pushing of the ground stronger -> body rises.
        self.stance_push_down_factor = self.hexapod().stance_push_down_factor  # Increase pushing down while "ST" phase.
        self.slowing_factor = self.hexapod().slowing_factor  # Make servo move slower during "TD" and "LO". Higher value -> stable but slow.
        self.stance_slowing_factor = self.hexapod().stance_slowing_factor  # Slow the servo speed during the "ST" phase to compensate leg slipping.
        self.kalman_R = self.hexapod().kalman_R  # Adjust the aggressiveness of voltage measurement noise filtering
        self.optimal_position = self.hexapod().optimal_position  # Optimal Knee servo angle to improve stability and reduce tilt.
        
        # Servo data parameters
        self.position_history = []  # Record servo position
        self.voltage_history = []  # Record servo voltage
        self.state_history = []  # Record leg state
        self.start_time = 0  # Start time of when hexapod starts moving
        self.diagnose_times = []  # When we recorded current data

        # Windowing (To prevent crashing or movement stopping)
        self.writing = False  # If TRUE -> Servo cant read data
        self.reading = False  # If TRUE -> Servo cant move
        
        # Servos children
        self.filter = Kalman(self, Board.getBusServoVin(servo_id))  # Voltage measurement noise filter
        self.graph = None  # Graph enabling to view joint data in real time.
        # But there is some issue with RPi... if using the matplotlib -> unable to save the joint data by "pickle"
        
        # Initial servo diagnose (to have reference values for the first run)
        self.diagnose(track=True)

    def __repr__(self) -> str:
        """
        This method is called when trying to print the joint object. Example: print(hexapod.rf.knee) outputs "message"
        :return: printed "message" summarizing the status of printed joint.
        """
        current_position = self._get_position()
        current_voltage = self._get_voltage()
        message = f"""{self.name}:
    Position = {current_position}, Voltage = {current_voltage[1] / 1000} V"""
        return message

    def get_joint_run_parameters(self):
        """
        Method returning the current parameters of this specific joint. Used for generating the "Run_parameters.txt" file.
        :return: Current joint parameters.
        """
        parameters = dict(theta=self.theta,
                          speed=self.speed,
                          reading_window=self.reading_window,
                          writing_window_factor=self.writing_window_factor,
                          up_v_threshold=self.up_v_threshold,
                          down_v_threshold=self.down_v_threshold,
                          stance_push_down_factor=self.stance_push_down_factor,
                          slowing_factor=self.slowing_factor,
                          stance_slowing_factor=self.stance_slowing_factor,
                          kalman_R=self.kalman_R,
                          optimal_position=self.optimal_position)
        return parameters

    def update_joint_parameters_from_hexapod(self):
        """
        Method that sets the joints parameters the same as the ones of the hexapod.
        Need to call this method after adjusting the hexapod parameters, otherwise it will have no effect.
        :return: None
        """
        # Servo control parameters at first inherited from hexapod, but possible to tweak subjectively
        self.theta = self.hexapod().theta
        self.speed = self.hexapod().speed
        self.reading_window = self.hexapod().reading_window
        self.writing_window_factor = self.hexapod().writing_recover_factor
        self.up_v_threshold = self.hexapod().up_v_threshold
        self.down_v_threshold = self.hexapod().down_v_threshold
        self.stance_push_down_factor = self.hexapod().stance_push_down_factor
        self.slowing_factor = self.hexapod().slowing_factor
        self.stance_slowing_factor = self.hexapod().stance_slowing_factor
        self.kalman_R = self.hexapod().kalman_R
        self.optimal_position = self.hexapod().optimal_position

    def adjust_voltage_thresholds(self):
        """
        Method that changes the voltage threshold based on how far the Knee angle is from the optimal Knee angle.
        Designed to compensate the tilting of the hexapod.
        :return: None
        """
        diff = abs(self.position_history[-1] - self.optimal_position)
        old_threshold = self.down_v_threshold
        new_threshold = old_threshold

        # The joint is pushing too much -> make it push less
        if self.position_history[-1] < self.optimal_position:
            new_threshold = old_threshold - diff*0.05  # factor 0.05 can be adjusted manually

        # The joint is not pushing enough -> make it push more
        if self.position_history[-1] > self.optimal_position:
            new_threshold = old_threshold + diff*0.05  # factor 0.05 can be adjusted manually

        # Adjust voltage only for knee servos.
        if self.type == "Knee":
            self.down_v_threshold = np.clip(round(new_threshold), 25, 40)  # Clipping values can be changed - these are my best
            # print(f"{self.name} {self.down_v_threshold}")

    def _get_position(self, track=False) -> int:
        """
        Method that reads the current position of joints servo.
        :param track: If is set true, the read position value is saved in joint position history to generate graphs etc.
        :return: Current joints servo position value. Int between 0 and 1000.
        """

        while any(joint.writing for joint in self.hexapod().all_joints):
            time.sleep(0.0001)
            # print(f"Can't read cause writing...")
            # Wait here until joint stops writing
            pass

        else:
            self.reading = True
            current_position = Board.getBusServoPulse(self.servo_id)  # Physically read the position value.
            # Save position value to position history.
            if track:
                # Correct the direction of servos for the joints in left side legs.
                if self.side == 'R':
                    self.position_history.append(current_position)
                else:
                    self.position_history.append(Joint.max_position - current_position)
            self.reading = False
            return current_position

    def _get_voltage(self, track=False) -> tuple:
        """
        Method that reads the current voltage across the joints servo.
        :param track: If set true, the read voltage value is saved in joint voltage history to generate graphs etc.
        :return: Current voltage across the joints servo. Should be Int between 9000 and 11000 [mV].
        """

        while any(joint.writing for joint in self.hexapod().all_joints):
            time.sleep(0.0001)
            # Wait here until joint stops writing
            pass

        else:
            self.reading = True
            current_voltage_r = Board.getBusServoVin(self.servo_id)  # Physically read the voltage value.
            current_voltage_f = int(self.filter.filter_out_noise(current_voltage_r))

            # Save voltage value to the voltage history
            if track:
                self.voltage_history.append([current_voltage_r, current_voltage_f])
            self.reading = False
            return current_voltage_r, current_voltage_f

    def diagnose(self, track=False) -> tuple:
        """
        Method that collect position and voltage data from the individual joint.
        :param track: If set true, collected data is appended to the corresponding history to generate graphs etc.
        :return: Tuple containing the current position and voltage value: (pos, voltage)
        """

        current_position = self._get_position(track=track)
        current_voltage = self._get_voltage(track=track)
        if track:
            self.state_history.append(self.parent().state)
        self.diagnose_times.append(time.time()-self.start_time)
        return current_position, current_voltage

    def set_position(self, theta: int, speed=None):
        """
        Method that physically moves the joints servo
        :param speed: Set to :Joint.speed by default
        :param theta: Desired position that the servo is supposed to reach. Value between 0 and 1000.
        :return: None
        """
        self.writing = True
        while any(joint.reading for joint in self.hexapod().all_joints):
            time.sleep(0.0001)
            # print("cant write cause reading")
            # Wait here until joint stops writing
            pass

        else:
            if self.side == 'R':
                theta = theta
            else:
                # For the left side joints, correct the theta value to its complementary value since the direction is reversed.
                theta = Joint.max_position - theta

            speed = self.speed if speed is None else speed
            # This physically moves the servo
            Board.setBusServoPulse(self.servo_id, theta, speed)
            time.sleep(speed / self.writing_window_factor)  # Time window to prevent reading stopping the servo
            self.writing = False
            time.sleep(self.reading_window)  # Reading window

    def move_until_voltage_threshold(self, direction='up', threshold=100, speed=500):
        """
        Method that moves the servo until voltage threshold is reached. Used for knee joints in "TD" and "LO" phase.
        This is the "Reflex Feedback Condition".
        :param direction: Decides if the servo is moving up or down. For "LO" phase use direction='up'. For "TD" phase use direction='down'.
        :param threshold: How big the difference between the reference voltage and the current voltage has to be in order to stop the servo movement.
        :param speed: How long it takes for the servo to reach the max position.
        :return: None
        """
        if direction != 'up' and direction != 'down':
            raise AttributeError('Wrong direction value. Try: "up" or "down"')

        voltage_reference = self.voltage_history[-1][1]  # Get the voltage before moving to compare and see the voltage difference.

        if direction == 'down':
            theta = 300  # Lowest position limit -> could be 0
        else:
            theta = 700  # Highest position limit -> could be 1000

        self.set_position(theta, speed)  # Move the servo to the lowest or highest position

        # Check if the servo reached the voltage difference and if yes, stop the servo movement
        while True:
            current_position = self.position_history[-1]
            current_voltage = self.voltage_history[-1][1]  # Get filtered voltage
            voltage_dif = abs(voltage_reference - current_voltage)
            if self.parent().name == 'LF':
                print(f"{self.name} {direction}: {voltage_dif}/{threshold}  ; {current_voltage}")
            time.sleep(0.0001)  # Recovery time
            if voltage_dif >= threshold:
                if self.parent().name == 'LF':
                    print(f"{self.name} {direction}: {voltage_dif}/{threshold}... Switching State from {self.parent().state}")
                self.set_position(current_position)  # Stop the servo at the current position
                return

    def save(self, path):
        """
        Method which saves the joint object. So its data could be used even after the program finishes.
        Outputs a file containing the joint object data in the "JointData" directory.
        :return: None
        """

        # Set up the directory to save the data to.
        os.makedirs(f'{path}/JointData', exist_ok=True)

        with open(f'{path}/JointData/{self.name}', 'wb') as file:
            # Save the data.
            pickle.dump(self, file)  # Pickle fails if graph to view joints realtime has been used.

    @staticmethod
    def load_joint(file_path):
        """
        Method that loads the saved joints
        :param file_path: Name of the joint which is meant to be loaded.
        :return: Loaded joint. The desired joint as an instance of the Joint class.
        """
        with open(file_path, 'rb') as file:
            loaded_joint = pickle.load(file)
        return loaded_joint


class Leg:
    """
    Class that combines three servo joints to make up a leg.
    This class offers tools to control, diagnose and manipulate a group of 3 servos (Shoulder, Knee and Paw).
    """

    def __init__(self, parent, name: str, shoulder: tuple, knee: tuple, paw: tuple, state='ST'):
        self.parent = ref(parent) if parent is not None else None  # Legs parent is the Hexapod
        self.name = name  # String naming the leg based on the position (RF, RM, RH, LF, LM, LH).
        self.state = state  # Current leg state (ST, LO, SW or TD)
        self.cycle = 0  # How many times has the leg been moved
        self.history = []  # History of all joint positions and voltages
        self.shoulder = Joint(self, shoulder[0], shoulder[1])  # Shoulder joint servo id
        self.knee = Joint(self, knee[0], knee[1])  # Knee joint servo id
        self.paw = Joint(self, paw[0], paw[1])  # Paw joint servo id
        self.joints = (self.shoulder, self.knee, self.paw)  # Tuple of leg joints.
        
    def __repr__(self) -> str:
        """
        This method is called when trying to print the leg object. Example: print(hexapod.rf) outputs "message"
        :return: printed "message" summarizing the status of the printed leg.
        """
        diagnosis = self.diagnose(track=False)
        message = f'''
        {self.name}  Status: {self.state}, Cycle: {self.cycle}
            Shoulder: Position = {diagnosis[2][0]}, Voltage = {diagnosis[3][0] / 1000} V
            Knee:     Position = {diagnosis[2][1]}, Voltage = {diagnosis[3][1] / 1000} V
            Paw:      Position = {diagnosis[2][2]}, Voltage = {diagnosis[3][2] / 1000} V'''
        return message

    def diagnose(self, track=False) -> tuple:
        """
        Method that collects the positions and voltage data for every joint in the leg.
        :param track: If set true, collected data is appended to the corresponding history of the joint.
        :return: Tuple containing:
            (name of the leg, current state of the leg, [shoulder_pos, knee_pos, paw_pos], [shoulder_v, knee_v, paw_v])
        """
        joint_positions = []  # List of positions per joint in leg: [shoulder_pos, knee_pos, paw_pos]
        joint_voltages = []  # List of voltages per joint in leg: [shoulder_v, knee_v, paw_v]

        for joint in self.joints:
            diagnosis = joint.diagnose(track=track)
            current_position = diagnosis[0]
            current_voltage = diagnosis[1]

            joint_positions.append(current_position)
            joint_voltages.append(current_voltage)

        diagnosis = (self.name, self.state, joint_positions, joint_voltages)
        return diagnosis

    def init_leg(self):
        """
        Sets every joint in leg to middle position
        :return: None
        """
        self.cycle = 0
        for joint in self.joints:
            Board.setBusServoPulse(joint.servo_id, 500, 300)
        time.sleep(300 / 1000)

    def set_state(self, state: str):
        """
        Method which sets the leg state based on the current control system.
        :param state: Desired state ('ST', 'LO', 'SW', 'TD')
        :return: None
        """
        if self.name == 'LF':
            print(f"Setting state: {state}")
        self.parent().control_system.set_state(self, state)

    def change_state(self):
        """
        Method that cycles through the states: ST -> LO -> SW -> TD -> ST ...
        :return: None
        """

        if self.state == 'ST':
            self.state = 'LO'

        elif self.state == 'LO':
            self.state = 'SW'

        elif self.state == 'SW':
            self.state = 'TD'

        elif self.state == 'TD':
            self.state = 'ST'

        self.cycle += 1  # Leg has moved
        self.set_state(self.state)  # Set the new state


class Hexapod:
    """
    Class that combines all legs together to represent the complete hexapod.
    This class offers tools to control, diagnose and manipulate the hexapod as a whole.
    User should interact with this class in order to operate the hexapod.
    """

    # Parameters to optimize  (if changed -> need to call the joint update parameters method to write the changed parameters to the joints)
    theta = 50  # Integer (0<theta<~150). How far the shoulder moves during Swing/Stance.
    speed = 500  # How long it takes in [ms] for servo to reach the desired position.
    reading_window = 0.5  # Float (0.1<x<0.7). During this time, servo can't move.
    writing_recover_factor = 100000  # No need to change... Time for servo to "recover" after moving command.
    up_v_threshold = 30  # Voltage threshold in [mV]. If reached "LO" -> "SW". Higher values -> Lifting leg higher -> body drops.
    down_v_threshold = 50  # Voltage threshold in [mV]. If reached "TD" -> "ST". Higher values -> Pushing of the ground stronger -> body rises.
    stance_push_down_factor = 1  # Increase pushing down while "ST" phase.
    slowing_factor = 100  # Make servo move slower during "TD" and "LO". Higher value -> stable but slow.
    stance_slowing_factor = 1  # Slow the servo speed during the "ST" phase to compensate leg slipping.
    kalman_R = 20  # Adjust the aggressiveness of voltage measurement noise filtering
    optimal_position = 625  # Optimal Knee servo position to improve stability and reduce tilt.

    def __init__(self, rf: tuple, rm: tuple, rh: tuple, lf: tuple, lm: tuple, lh: tuple):
        # Hexapod ID parameters
        self.control_system = cs.ReflexBased(self)
        self.morphology = mc.leg_initializer(self)
        self.all_joints = []
        self.history = []
        
        self.rf = Leg(self, rf[0], rf[1], rf[2], rf[3])  # Right front leg
        self.rm = Leg(self, rm[0], rm[1], rm[2], rm[3])  # Right middle leg
        self.rh = Leg(self, rh[0], rh[1], rh[2], rh[3])  # Right hind leg
        self.lf = Leg(self, lf[0], lf[1], lf[2], lf[3])  # Left front leg   
        self.lm = Leg(self, lm[0], lm[1], lm[2], lm[3])  # Left middle leg
        self.lh = Leg(self, lh[0], lh[1], lh[2], lh[3])  # Left hind leg
        self.all_legs = (self.lh, self.lm, self.lf, self.rh, self.rm, self.rf)
        
        self.start_time = time.time()
        self.run_personal_comment = None
        
        # Gather all joints in one list
        for leg in self.all_legs:
            for joint in leg.joints:
                self.all_joints.append(joint)

    def __repr__(self):
        output = f"""
control system: {self.control_system.name}
RF: {self.rf.state}
RM: {self.rm.state}
RH: {self.rh.state}
LF: {self.lf.state}
LM: {self.lm.state}
LH: {self.lh.state}
"""
        return output

    def get_hexapod_run_parameters(self):
        parameters = dict(
            control_system=self.control_system.name,
            morphology=self.morphology.name,
            theta=self.theta,
            speed=self.speed,
            reading_window=self.reading_window,
            writing_window_factor=self.writing_recover_factor,
            up_v_threshold=self.up_v_threshold,
            down_v_threshold=self.down_v_threshold,
            stance_push_down_factor=self.stance_push_down_factor,
            slowing_factor=self.slowing_factor,
            stance_slowing_factor=self.stance_slowing_factor,
            kalman_R=self.kalman_R,
            optimal_position=self.optimal_position,
            run_comment=self.run_personal_comment)
        return parameters

    def update_joints_parameters(self):
        for joint in self.all_joints:
            joint.update_joint_parameters_from_hexapod()
        print("Joint parameters updated successfully!")

    def init_legs(self):
        """
        Method that sets all legs to the default position, where each servo is in its middle position (500)
        :return: None
        """
        print("Initialising legs...")
        joint_init_angles = self.morphology.init_legs()
        for joint in self.all_joints:
            joint.init_angle = joint_init_angles[joint.name]
            print(f"{joint.name} initial angle: {joint.init_angle}")
        print("All legs initialised!")
        time.sleep(1)

    def set_control_system(self, control_system):
        """
        Method which sets the control system to reflex based.
        Reflex based uses voltage threshold to complete the "Lift Off" and "Touch Down" states.
        :return: None
        """
        self.control_system = control_system(self)
        print(f"Control system has been set to: {control_system.name}")

    def set_init_config(self, init_config):
        self.morphology = init_config(self)
        print(f"Initial configuration has been set to: {init_config.name}")

    # FIXME enable and disable graph is for debugging purposes
    def enable_graph(self):
        print("Real time graph has been enabled!")
        for joint in self.all_joints:
            joint.graph = RealTimeGraph(joint)

    def disable_graph(self):
        # FIXME if raspberry pi uses matplotlib in graph, it fails to pickle joints properly and no .html graphs can be created
        print("Real time graph has been disabled!")
        for joint in self.all_joints:
            joint.graph = None

    def state_preset(self, state_preset: list):
        """
        Method that sets desired state configuration for each of hexapods legs based on the given list of states.
        :param state_preset: list of desired states. 
            Order: RF, RM, RH, LF, LM, LH. Possible values: 'ST', 'LO', 'SW', 'TD'.
        :return:
        """
        leg_index = 0

        # Check if the input has the correct length
        if len(state_preset) != 6:  # Only 6 states are allowed
            raise f"Input has incorrect length. Correct length is 6 (one state for each leg)"

        else:
            # Check if the input has correct stance types
            for state in state_preset:
                if state != 'ST' and state != 'LO' and state != 'SW' and state != 'TD':
                    raise AttributeError("Wrong state values")

            print("Setting the preset state...")
            # Set the desired states for the legs
            for state in state_preset:
                self.all_legs[leg_index].state = state
                leg_index += 1

            print("State preset set successfully!")

    def diagnose(self, track=False) -> tuple:
        """
        Method that collects position and voltage data from each leg (each leg collect data from each joint).
        :param track: If set true, every joint appends collected data to its corresponding history to generate graphs etc.
        :return: Tuple with List of tuples, each containing data from each leg and how long it took to diagnose:
        (leg name, leg state, [shoulder_pos, knee_pos, paw_pos], [shoulder_volt, knee_volt, paw_volt]), duration [s]
        """
        diagnosis = []
        start_time = time.time()
        for leg in self.all_legs:
            diagnosis.append(leg.diagnose(track=track))
        if track:
            self.history.append(diagnosis)
        end_time = time.time()
        dur = round(end_time - start_time, 3)
        return diagnosis, dur

    def save(self):
        """
        Method that saves all joint objects in the "JointData" directory,
        so they can be accessed even after the program finishes.
        :return: None
        """
        # Prepare the folders to save graphs in
        os.makedirs('RunData', exist_ok=True)

        # Prepare unique folder for the current graph
        if not os.path.exists(f"RunData/Run_1"):
            this_run_path = f"RunData/Run_1"
        else:
            existing_runs = [x[1] for x in os.walk("RunData/")][0]
            existing_indexes = []
            for item in existing_runs:
                list_of_indexes = [i for i in item if i.isdigit()]
                index = int("".join(list_of_indexes))
                existing_indexes.append(index)
            last_run_index = max(existing_indexes)
            this_run_path = f"RunData/Run_{last_run_index + 1}"
        os.mkdir(this_run_path)

        print("Saving hexapod data...")
        for joint in self.all_joints:
            joint.save(this_run_path)

        with open(f'{this_run_path}/Hexapod', 'wb') as file:
            pickle.dump(self, file)
        print(f"Hexapod data has been saved successfully! {this_run_path}")

    @staticmethod
    def load_hexapod(filepath):
        """
        Method that loads the saved hexapod
        :return: Loaded hexapod.
        """
        with open(filepath, 'rb') as file:
            loaded_hexapod = pickle.load(file)
        return loaded_hexapod

    def move(self, duration=10, measure=False, speed=200, joints_to_view: Joint or list = None):
        print(f"Control system: {self.control_system.name}")
        Joint.speed = speed
        self.start_time = time.time()
        for joint in self.all_joints:
            joint.start_time = self.start_time
        self.control_system.move(self, duration, measure, joints_to_view)
        self.run_personal_comment = str(input("Add personal comment: "))
        print("Hexapod successfully finished moving!")


class Constructor:
    """
    Static class used to construct the hexapod. Is not meant to have instances.
    Use the "construct_hexapod" method to create the hexapod object.
    Construct joints -> load them to corresponding legs -> mount legs to the hexapod.
    R... right
    L... left
    F... front
    M... middle
    H... hind

    shoulder (move left/right)
    knee (move up/down)
    paw (move up/down)
    """
    # Construct joints. Assign servo ID to each joint according to the Hiwonder Manual
    LH_shoulder = (1, 'Shoulder')
    LH_knee = (2, 'Knee')
    LH_paw = (3, 'Paw')

    LM_shoulder = (4, 'Shoulder')
    LM_knee = (5, 'Knee')
    LM_paw = (6, 'Paw')

    LF_shoulder = (7, 'Shoulder')
    LF_knee = (8, 'Knee')
    LF_paw = (9, 'Paw')

    RH_shoulder = (10, 'Shoulder')
    RH_knee = (11, 'Knee')
    RH_paw = (12, 'Paw')

    RM_shoulder = (13, 'Shoulder')
    RM_knee = (14, 'Knee')
    RM_paw = (15, 'Paw')

    RF_shoulder = (16, 'Shoulder')
    RF_knee = (17, 'Knee')
    RF_paw = (18, 'Paw')

    # Construct legs
    rf = ('RF', RF_shoulder, RF_knee, RF_paw)
    rm = ('RM', RM_shoulder, RM_knee, RM_paw)
    rh = ('RH', RH_shoulder, RH_knee, RH_paw)

    lf = ('LF', LF_shoulder, LF_knee, LF_paw)
    lm = ('LM', LM_shoulder, LM_knee, LM_paw)
    lh = ('LH', LH_shoulder, LH_knee, LH_paw)

    @staticmethod
    def construct_hexapod():
        return Hexapod(Constructor.rf, Constructor.rm, Constructor.rh, Constructor.lf, Constructor.lm, Constructor.lh)


if __name__ == '__main__':

    # Construct 
    hexapod = Constructor.construct_hexapod()
    
    hexapod.set_control_system(cs.ReflexBased)

    hexapod.set_init_config(mc.OPT)

    hexapod.init_legs()

    # hexapod.enable_graph()  # Enabling graph crashes the ability to save hexapod data "pickle"
    available_leg_states = ['LO', 'ST', 'SW', 'TD']
    init_state = [random.choice(available_leg_states),
                  random.choice(available_leg_states),
                  random.choice(available_leg_states),
                  random.choice(available_leg_states),
                  random.choice(available_leg_states),
                  random.choice(available_leg_states)]

    random.shuffle(init_state)
    hexapod.state_preset(init_state)
    hexapod.rf.knee.adjust_voltage_thresholds()
    # Parameters to optimize
    hexapod.theta = 40
    hexapod.speed = 400
    hexapod.reading_window = 0.25
    hexapod.writing_recover_factor = 100000
    hexapod.up_v_threshold = 500
    hexapod.down_v_threshold = 260
    hexapod.stance_push_down_factor = 1.5
    hexapod.slowing_factor = 30  # For Lift Off and Push Down states  # Change name to "knee_slowing_factor"
    hexapod.stance_slowing_factor = 8  # Change name to "shoulder_slowing_factor"
    hexapod.kalman_R = 20
    hexapod.optimal_position = 625

    hexapod.update_joints_parameters()
    
    input("Press ENTER to Start")
    print("""
------------------------  START MOVING  -------------------------
""")
    time.sleep(1)
    hexapod.move(30, measure=True, joints_to_view=[hexapod.lf.knee])
    hexapod.save()
