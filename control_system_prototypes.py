import threading
import random
import time
from weakref import ref

"""
THIS FILE IS NOT USED IN THE PROGRAM. ITS THE CONTROL SYSTEM PROTOTYPES I TRIED IN ORDER TO IMPROVE THE HEXAPOD MOVEMENT 
PERFORMANCE
"""


class ControlSystem:
    name = 'Control System Static Class'
    max_position = 1000

    def __init__(self, parent):
        self.parent = ref(parent)

    @staticmethod
    def _rb_state(leg, state):
        theta = 500 - leg.knee.theta
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                desired_shoulder_angle = theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'LO':
                # LIFT OFF
                leg.knee.move_until_voltage_threshold(direction='up', threshold=63, step=10)
                leg.state = state

            elif state == 'SW':
                # SWING
                desired_shoulder_angle = ControlSystem.max_position - theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'TD':
                # TOUCH DOWN
                leg.knee.move_until_voltage_threshold(direction='down', threshold=90, step=40)
                leg.state = state

        else:
            raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def _rb_state_v2(leg, state):
        theta = 500 - leg.knee.theta
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                desired_shoulder_angle = theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'LO':
                threshold = leg.knee.up_v_threshold
                slowing_factor = leg.knee.stance_slowing_factor
                speed = leg.knee.speed * slowing_factor
                # LIFT OFF
                leg.knee.move_until_voltage_threshold(direction='up', threshold=threshold, speed=speed)
                leg.state = state

            elif state == 'SW':
                # SWING
                desired_shoulder_angle = ControlSystem.max_position - theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'TD':
                threshold = leg.knee.down_v_threshold
                slowing_factor = leg.knee.stance_slowing_factor
                speed = leg.knee.speed * slowing_factor
                # TOUCH DOWN
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)
                leg.state = state

        else:
            raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def _rb_state_v3(leg, state):
        theta = 500 - leg.knee.theta
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                threshold = leg.knee.down_v_threshold * leg.knee.stance_push_down_factor
                slowing_factor = leg.knee.stance_slowing_factor
                speed = leg.knee.speed * slowing_factor
                desired_shoulder_angle = theta
                leg.shoulder.set_position(desired_shoulder_angle)
                # Push down while moving back
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)
                leg.state = state

            elif state == 'LO':
                # LIFT OFF
                threshold = leg.knee.up_v_threshold
                slowing_factor = leg.knee.stance_slowing_factor
                speed = leg.knee.speed * slowing_factor
                leg.knee.move_until_voltage_threshold(direction='up', threshold=threshold, speed=speed)
                leg.state = state

            elif state == 'SW':
                # SWING
                desired_shoulder_angle = ControlSystem.max_position - theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'TD':
                # TOUCH DOWN
                threshold = leg.knee.down_v_threshold
                slowing_factor = leg.knee.stance_slowing_factor
                speed = leg.knee.speed * slowing_factor
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)
                leg.state = state

        else:
            raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def _rb_state_v4(leg, state):
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                threshold = leg.knee.down_v_threshold * leg.knee.stance_push_down_factor
                speed = leg.knee.speed * leg.knee.stance_slowing_factor
                desired_shoulder_angle = leg.shoulder.init_angle - leg.shoulder.theta
                leg.shoulder.set_position(desired_shoulder_angle, speed=speed)
                # Push down while moving back
                speed = int(leg.knee.speed * leg.knee.slowing_factor / 1)
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)
                leg.state = state

            elif state == 'LO':
                # LIFT OFF
                threshold = leg.knee.up_v_threshold
                slowing_factor = leg.knee.slowing_factor
                speed = leg.knee.speed * slowing_factor
                leg.knee.move_until_voltage_threshold(direction='up', threshold=threshold, speed=speed)
                leg.state = state

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
                leg.knee.move_until_voltage_threshold(direction='down', threshold=threshold, speed=speed)
                leg.state = state

        else:
            raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def _ab_state(leg, state):
        theta = 500 - leg.knee.theta
        if state == 'ST' or state == 'LO' or state == 'SW' or state == 'TD':
            if state == 'ST':
                # STANCE
                desired_shoulder_angle = theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'LO':
                # LIFT OFF
                leg.knee.set_position(550)
                leg.state = state

            elif state == 'SW':
                # SWING
                desired_shoulder_angle = ControlSystem.max_position - theta
                leg.shoulder.set_position(desired_shoulder_angle)
                leg.state = state

            elif state == 'TD':
                # TOUCH DOWN
                leg.knee.set_position(450)
                leg.state = state
            else:
                raise "Wrong desired state value. Try: 'ST', 'LO', 'SW' or 'TD'."

    @staticmethod
    def _move_leg(leg, duration):

        start_time = time.time()
        current_time = 0
        while current_time < duration:
            current_time = round(time.time() - start_time, 3)
            leg.change_state()
            # print(f"{leg.name} time: {current_time}, state: {leg.state}")

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
    def measure_all_joints(hexapod, threads, joints_to_view):
        durations = []
        while any(thread.is_alive() for thread in threads):
            diagnosis_duration = hexapod.diagnose(track=True)[1]
            durations.append(diagnosis_duration)
            print(f"Diagnose took {diagnosis_duration} s")
            ControlSystem._plot_realtime(joints_to_view)
        return durations

    @staticmethod
    def measure_lo_td(hexapod, threads, joints_to_view):
        # While at least one of the leg is still running conduct the measurement
        durations = []
        while any(thread.is_alive() for thread in threads):
            for leg in hexapod.all_legs:
                # Partial diagnose necessary for Lift Off and Touch Down states
                if leg.state == 'LO' or leg.state == 'TD':
                    start_time = time.time()
                    leg.knee.diagnose(track=True)  # Diagnose only joints in need
                    end_time = time.time()
                    diagnosis_duration = round(end_time - start_time, 3)
                    durations.append(diagnosis_duration)
                    print(f"Diagnose took {diagnosis_duration} s")
                    ControlSystem._plot_realtime(joints_to_view)
        return durations

    @staticmethod
    def measure_lo_td_st(hexapod, threads, joints_to_view):
        # While at least one of the leg is still running conduct the measurement
        durations = []
        while any(thread.is_alive() for thread in threads):
            for leg in hexapod.all_legs:
                # Partial diagnose necessary for Lift Off and Touch Down states
                if leg.state == 'LO' or leg.state == 'TD' or leg.state == 'ST':
                    start_time = time.time()
                    leg.knee.diagnose(track=True)  # Diagnose only joints in need
                    end_time = time.time()
                    diagnosis_duration = round(end_time - start_time, 3)
                    durations.append(diagnosis_duration)
                    print(f"Diagnose of {leg.name} took {diagnosis_duration} s")
                    ControlSystem._plot_realtime(joints_to_view)
        return durations

    @staticmethod
    def measure_knee(hexapod, threads, joints_to_view):
        # While at least one of the leg is still running conduct the measurement
        durations = []
        while any(thread.is_alive() for thread in threads):
            start_time = time.time()
            for joint in hexapod.all_joints:
                if joint.type == 'Knee':
                    joint.diagnose(track=True)
                    joint.adjust_voltage_thresholds()
            end_time = time.time()
            diagnosis_duration = round(end_time - start_time, 3)
            durations.append(diagnosis_duration)
            # print(f"Diagnose of joints took {diagnosis_duration} s")
            ControlSystem._plot_realtime(joints_to_view)
        return durations

    @staticmethod
    def measure_knee_and_shoulder(hexapod, threads, joints_to_view):
        # While at least one of the leg is still running conduct the measurement
        durations = []
        while any(thread.is_alive() for thread in threads):
            start_time = time.time()
            for joint in hexapod.all_joints:
                if joint.type == 'Knee' or joint.type == 'Shoulder':
                    joint.diagnose(track=True)
            end_time = time.time()
            diagnosis_duration = round(end_time - start_time, 3)
            durations.append(diagnosis_duration)
            # print(f"Diagnose of joints took {diagnosis_duration} s")
            ControlSystem._plot_realtime(joints_to_view)
        return durations

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


# Reflex based, dependent legs
class RB_dependent_v1(ControlSystem):
    # FIXME
    name = "RB_dependent_v1"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joint_to_view=None):

        cycle = 0

        while cycle < num_of_cycles:
            cycle += 1

            print(f"cycle {cycle} ---------------------------------------------------")

            for leg in hexapod.all_legs:
                leg.change_state()
                if measure:
                    # Hexapod conducts diagnose after the desired state is acquired
                    # Each joint saves the collected data.
                    hexapod.history.append(leg.diagnose(track=True))
                    ControlSystem._plot_realtime(joint_to_view)


# Reflex based, dependent legs, leg completes full cycle
class RB_dependent_v2(ControlSystem):
    # FIXME
    name = "RB_dependent_v2"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joint_to_view=None):

        cycle = 0

        while cycle < num_of_cycles:
            cycle += 1

            print(f"cycle {cycle} ---------------------------------------------------")

            for leg in hexapod.all_legs:
                if leg.state == 'TD':
                    leg.change_state()
                    if measure:
                        # Hexapod conducts diagnose after the desired state is acquired
                        # Each joint saves the collected data.
                        hexapod.history.append(leg.diagnose(track=True))
                        ControlSystem._plot_realtime(joint_to_view)

                else:
                    while leg.state != 'TD':
                        leg.change_state()
                        if measure:
                            # Hexapod conducts diagnose after the desired state is acquired
                            # Each joint saves the collected data.
                            hexapod.history.append(leg.diagnose(track=True))
                            ControlSystem._plot_realtime(joint_to_view)


# Reflex based, hybrid legs, leg completes cycle dependently, stance independently all legs at the same time
class RB_dependent_v3(ControlSystem):
    # FIXME
    name = "RB_dependent_v3"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joint_to_view=None):
        cycle = 0

        while cycle < num_of_cycles:
            cycle += 1

            print(f"cycle {cycle} ---------------------------------------------------")
            threads = []
            for leg in hexapod.all_legs:
                leg_thread = threading.Thread(target=leg.change_state)
                threads.append(leg_thread)
            if all(leg.state == 'TD' for leg in hexapod.all_legs):
                for thread in threads:
                    thread.start()
                for thread in threads:
                    thread.join()

                if measure:
                    # Hexapod conducts diagnose after the desired state is acquired
                    # Each joint saves the collected data.
                    hexapod.history.append(hexapod.diagnose(track=True))
                    ControlSystem._plot_realtime(joint_to_view)

            else:
                for leg in hexapod.all_legs:
                    while leg.state != 'TD':
                        leg.change_state()
                        if measure:
                            # Hexapod conducts diagnose after the desired state is acquired
                            # Each joint saves the collected data.
                            hexapod.history.append(leg.diagnose(track=True))
                            ControlSystem._plot_realtime(joint_to_view)


# Reflex based, dependent legs, move random legs
class RB_dependent_v4(ControlSystem):
    # FIXME
    name = "RB_dependent_v4"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joint_to_view=None):

        cycle = 0

        if joint_to_view is not None:
            joint_to_view.graph.open_figure()

        while cycle < num_of_cycles:
            cycle += 1

            print(f"cycle {cycle} ---------------------------------------------------")

            random.shuffle(list(hexapod.all_legs))
            for leg in hexapod.all_legs:
                leg.change_state()
                if measure:
                    # Hexapod conducts diagnose after the desired state is acquired
                    # Each joint saves the collected data.
                    hexapod.history.append(leg.diagnose(track=True))
                    if joint_to_view is not None:
                        joint_to_view.graph.update(joint_to_view)
                        joint_to_view.graph.show()


# ANGLE BASED CONTROL SYSTEM IS MEANT MOSTLY FOR DEBUGGING.

# Angle based, dependent legs
class AB_dependent_v1(ControlSystem):
    name = "AB_dependent_v1"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._ab_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joint_to_view=None):

        cycle = 0

        while cycle < num_of_cycles:
            cycle += 1

            print(f"cycle {cycle} ---------------------------------------------------")

            for leg in hexapod.all_legs:
                leg.change_state()
                # Hexapod conducts diagnose after the desired state is acquired
                # Each joint saves the collected data.
                if measure:
                    hexapod.history.append(leg.diagnose(track=True))
                    ControlSystem._plot_realtime(joint_to_view)


# Angle based, independent legs
class AB_independent_v1(ControlSystem):
    name = "AB_independent_v1"

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._ab_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        # Build and start leg threads
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_all_joints(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Partial diagnose on each turn, rb_state
class RB_independent_v1(ControlSystem):
    name = 'RB_independent_v1'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_all_joints(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Full diagnose on each turn rb_state
class RB_independent_v2(ControlSystem):
    name = 'RB_independent_v2'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_all_joints(hexapod, threads, joints_to_view)

        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Full diagnose on each turn, rb_state_v2
class RB_independent_v3(ControlSystem):
    name = 'RB_independent_v3'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v2(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_all_joints(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Partial diagnose on each turn, rb_state_v2
class RB_independent_v4(ControlSystem):
    name = 'RB_independent_v4'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v2(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_lo_td(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Partial diagnose on each turn, rb_state_v3... push down while in STANCE
class RB_independent_v5(ControlSystem):
    name = 'RB_independent_v5'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v3(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_lo_td_st(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


# Partial diagnose on each turn, rb_state_v4... push down while in STANCE, slower stance
class RB_independent_v6(ControlSystem):
    name = 'RB_independent_v6'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v4(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        ControlSystem.measure_lo_td_st(hexapod, threads, joints_to_view)
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


#  rb_state_v4... push down while in STANCE, slower stance
class RB_independent_v7(ControlSystem):
    name = 'RB_independent_v7'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v4(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        durations = ControlSystem.measure_all_joints(hexapod, threads, joints_to_view)
        print(f"Average diagnose duration {round(sum(durations) / len(durations), 3)}")
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


#  rb_state_v4... push down while in STANCE, slower stance
class RB_independent_v8(ControlSystem):
    name = 'RB_independent_v8'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v4(hexapod, state)

    @staticmethod
    def move(hexapod, num_of_cycles=1, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, num_of_cycles)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        durations = ControlSystem.measure_knee_and_shoulder(hexapod, threads, joints_to_view)
        print(f"Average diagnose duration {round(sum(durations) / len(durations), 3)}")
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()


#  rb_state_v4... push down while in STANCE, slower stance
class RB_independent_v9(ControlSystem):
    name = 'RB_independent_v9'

    @staticmethod
    def set_state(hexapod, state: str):
        ControlSystem._rb_state_v4(hexapod, state)

    @staticmethod
    def move(hexapod, duration, measure=False, joints_to_view=None):
        threads = ControlSystem.start_leg_threads(hexapod, duration)

        '''----------------------------- Main Loop while legs are moving ----------------------------------'''
        durations = ControlSystem.measure_knee(hexapod, threads, joints_to_view)
        print(f"Average diagnose duration {round(sum(durations) / len(durations), 3)}")
        '''-------------------------------------- Main Loop stop ------------------------------------------'''

        for thread in threads:
            thread.join()

# Shortcuts... commented ones dont work yet
# v1 = RB_dependent_v1
# v2 = RB_dependent_v2
# v3 = RB_dependent_v3
# v4 = RB_dependent_v4
v5 = AB_dependent_v1
v6 = AB_independent_v1
v7 = RB_independent_v1
v8 = RB_independent_v2
v9 = RB_independent_v3
v10 = RB_independent_v4
v11 = RB_independent_v5
v12 = RB_independent_v6
v13 = RB_independent_v7
v14 = RB_independent_v8
v15 = RB_independent_v9
