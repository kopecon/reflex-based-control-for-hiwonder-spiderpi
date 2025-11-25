import sys
import threading
import time
from weakref import ref


class ControlSystem:
    name = 'Control System Static Class'
    max_position = 1000

    def __init__(self, parent):
        self.parent = ref(parent)

    @staticmethod
    def _plot_realtime(joint_to_view):
        if joint_to_view is not None:
            if type(joint_to_view) is list:
                for joint in joint_to_view:
                    if joint.graph is not None:
                        joint.graph.plot_realtime()
            else:
                if joint_to_view.graph is not None:
                    joint_to_view.graph.plot_realtime()

    @staticmethod
    def measure_all_joints(hexapod, joints_to_view):
        diagnosis_duration = hexapod.diagnose(track=True)[1]
        print(f"Diagnose took {diagnosis_duration} s")
        ControlSystem._plot_realtime(joints_to_view)
        return diagnosis_duration

    @staticmethod
    def measure_lo_td(hexapod):
        for leg in hexapod.all_legs:
            # Partial diagnose necessary for Lift Off and Touch Down states
            if leg.state == 'LO' or leg.state == 'TD':
                start_time = time.time()
                leg.knee.diagnose(track=True)  # Diagnose only joints in need
                end_time = time.time()
                diagnosis_duration = round(end_time - start_time, 3)
                print(f"Diagnose took {diagnosis_duration} s")
                return diagnosis_duration

    @staticmethod
    def measure_lo_td_st(hexapod):
        for leg in hexapod.all_legs:
            # Partial diagnose necessary for Lift Off and Touch Down states
            if leg.state == 'LO' or leg.state == 'TD' or leg.state == 'ST':
                start_time = time.time()
                leg.knee.diagnose(track=True)  # Diagnose only joints in need
                end_time = time.time()
                diagnosis_duration = round(end_time - start_time, 3)
                print(f"Diagnose of {leg.name} took {diagnosis_duration} s")
                return diagnosis_duration

    @staticmethod
    def measure_knee(hexapod):
        if sys.platform == 'win32':
            time.sleep(0.1)
        # While at least one of the leg is still running conduct the measurement
        start_time = time.time()
        for joint in hexapod.all_joints:
            if joint.type == 'Knee':
                joint.diagnose(track=True)
                joint.adjust_voltage_thresholds()
        end_time = time.time()
        diagnosis_duration = round(end_time - start_time, 3)
        # print(f"Diagnose of joints took {diagnosis_duration} s")
        return diagnosis_duration

    @staticmethod
    def measure_knee_and_shoulder(hexapod):
        start_time = time.time()
        for joint in hexapod.all_joints:
            if joint.type == 'Knee' or joint.type == 'Shoulder':
                joint.diagnose(track=True)
        end_time = time.time()
        diagnosis_duration = round(end_time - start_time, 3)
        # print(f"Diagnose of joints took {diagnosis_duration} s")
        return diagnosis_duration

    @staticmethod
    def _move_leg(leg, duration):

        start_time = time.time()
        current_time = 0
        while current_time < duration:
            current_time = round(time.time() - start_time, 3)
            leg.change_state()
            if leg.name == 'LF':
                # print(f"{leg.name} time: {current_time}, state: {leg.state}")
                pass

    @staticmethod
    def start_leg_threads(hexapod, duration):
        threads = []
        # Build and start leg threads
        for leg in hexapod.all_legs:
            leg_thread = threading.Thread(target=ControlSystem._move_leg, args=(leg, duration))
            threads.append(leg_thread)
            leg_thread.start()
        return threads

    @staticmethod
    def set_state(hexapod, state: str):
        raise NotImplemented

    @staticmethod
    def move(hexapod, duration, measure=False, joint_to_view=None):
        raise NotImplemented


class ReflexBased(ControlSystem):
    name = 'ReflexBased'

    @staticmethod
    def set_state(leg, state: str):
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                threshold = leg.knee.down_v_threshold * leg.knee.stance_push_down_factor
                speed = leg.knee.speed * leg.knee.stance_slowing_factor
                desired_shoulder_angle = leg.shoulder.init_angle - leg.shoulder.theta
                leg.shoulder.set_position(desired_shoulder_angle, speed=speed)
                # Push down while moving back
                speed = int(leg.knee.speed * leg.knee.slowing_factor / 1)
                leg.state = state
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)

            elif state == 'LO':
                # LIFT OFF
                threshold = leg.knee.up_v_threshold
                slowing_factor = leg.knee.slowing_factor
                speed = leg.knee.speed * slowing_factor
                leg.state = state
                leg.knee.move_until_voltage_threshold(direction='up', threshold=threshold, speed=speed)

            elif state == 'SW':
                # SWING
                desired_shoulder_angle = leg.shoulder.init_angle + leg.shoulder.theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'TD':
                # TOUCH DOWN
                threshold = leg.knee.down_v_threshold
                slowing_factor = leg.knee.slowing_factor
                speed = leg.knee.speed * slowing_factor
                leg.state = state
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)

        else:
            raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def move(hexapod, duration, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, duration)
        diagnose_durations = []

        while any(thread.is_alive() for thread in threads):
            '''----------------------------- Main Loop while legs are moving ----------------------------------'''
            # Conduct measurements (it is possible to choose different measurement options)
            diagnose_duration = ControlSystem.measure_knee(hexapod)
            diagnose_durations.append(diagnose_duration)
            ControlSystem._plot_realtime(joints_to_view)
            '''-------------------------------------- Main Loop stop ------------------------------------------'''
        print(f"Average diagnose duration {round(sum(diagnose_durations) / len(diagnose_durations), 3)}")
        for thread in threads:
            thread.join()
