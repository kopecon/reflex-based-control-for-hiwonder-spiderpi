from weakref import ref
from numpy import pi
import threading


def _rads_to_joint_position(rads):
    """
    Function that converts angle in radians into servo position values from 0 to 1000
    :param rads: angle in radians.
    :return: equivalent position of the servo (0~1000)
    """
    if rads > pi / 2 or rads < -pi / 2:
        raise ValueError('Angle should be in range -pi/2 ~ pi/2')
    joint_position = 500 + 1000 / pi * rads
    return int(joint_position)


def _shoulder_angle_calculation(rads):
    """
    This function compensates the physical structure of the robot in order to match
    the structure of the robot in Sato's simulation.
    :param rads: Desired angle of shoulders in radians
    :return: Equivalent position of shoulders
    """

    joint_position = _rads_to_joint_position(rads)

    front_shoulders = joint_position - _rads_to_joint_position(-pi / 3)
    mid_shoulders = joint_position
    hind_shoulders = joint_position + _rads_to_joint_position(-pi / 3)

    # Check for physical limits of servos
    front_shoulders = front_shoulders if front_shoulders > 0 else 0
    mid_shoulders = mid_shoulders if mid_shoulders > 0 else 0
    hind_shoulders = hind_shoulders if hind_shoulders > 0 else 0

    front_shoulders = front_shoulders if front_shoulders < 1000 else 1000
    mid_shoulders = mid_shoulders if mid_shoulders < 1000 else 1000
    hind_shoulders = hind_shoulders if hind_shoulders < 1000 else 1000

    return int(front_shoulders), int(mid_shoulders), int(hind_shoulders)


class leg_initializer:
    name = 'Default configuration'

    def __init__(self, parent):
        self.parent = ref(parent)
        self.speed = 400
        self.init_angles = {}
        self.update_init_angles(500, 500, 500, 500, 500, 500, 500, 500, 500)

    def update_init_angles(self,
                           hind_shoulders, hind_knees, hind_paws,
                           mid_shoulders, mid_knees, mid_paws,
                           front_shoulders, front_knees, front_paws):

        self.init_angles = {'LH Shoulder': hind_shoulders,
                            'LH Knee': hind_knees,
                            'LH Paw': hind_paws,
                            'LM Shoulder': mid_shoulders,
                            'LM Knee': mid_knees,
                            'LM Paw': mid_paws,
                            'LF Shoulder': front_shoulders,
                            'LF Knee': front_knees,
                            'LF Paw': front_paws,
                            'RH Shoulder': hind_shoulders,
                            'RH Knee': hind_knees,
                            'RH Paw': hind_paws,
                            'RM Shoulder': mid_shoulders,
                            'RM Knee': mid_knees,
                            'RM Paw': mid_paws,
                            'RF Shoulder': front_shoulders,
                            'RF Knee': front_knees,
                            'RF Paw': front_paws}

    def update_knees(self, knee_theta):
        for item in self.init_angles:
            if 'Knee' in item:
                self.init_angles[item] = knee_theta

            if 'Paw' in item:
                self.init_angles[item] = knee_theta  # For now, paw theta is the same as knee

    def _init_joint(self, joint, angles):
        joint.set_position(angles[joint.name], self.speed)

    def init_legs(self):
        threads = []
        # Build and start leg threads
        for joint in self.parent().all_joints:
            joint_thread = threading.Thread(target=self._init_joint, args=(joint, self.init_angles))
            threads.append(joint_thread)
            joint_thread.start()
        return self.init_angles


# ---------------------------------------------------------------------------------------------------  Knees angle 0 rad
class Sato1(leg_initializer):
    name = 'Sato1 - F0M0H0'

    def __init__(self, parent):
        super().__init__(parent)
        knees = _rads_to_joint_position(0 * pi)
        paws = knees
        front_shoulders, mid_shoulders, hind_shoulders = _shoulder_angle_calculation(0 * pi)
        self.update_init_angles(hind_shoulders, knees, paws,
                                mid_shoulders, knees, paws,
                                front_shoulders, knees, paws)


class Sato2(leg_initializer):
    name = 'Sato2 - F0M0H-pi6'

    def __init__(self, parent):
        super().__init__(parent)
        knees = _rads_to_joint_position(0 * pi)
        paws = knees
        front_shoulders, mid_shoulders, _ = _shoulder_angle_calculation(0 * pi)
        _, _, hind_shoulders = _shoulder_angle_calculation(-pi / 6)
        self.update_init_angles(hind_shoulders, knees, paws,
                                mid_shoulders, knees, paws,
                                front_shoulders, knees, paws)


class Sato3(leg_initializer):
    name = 'Sato3 - F0M-pi6H-pi6'

    def __init__(self, parent):
        super().__init__(parent)
        knees = _rads_to_joint_position(0 * pi)
        paws = knees
        front_shoulders, _, _ = _shoulder_angle_calculation(0 * pi)
        _, mid_shoulders, hind_shoulders = _shoulder_angle_calculation(-pi / 6)
        self.update_init_angles(hind_shoulders, knees, paws,
                                mid_shoulders, knees, paws,
                                front_shoulders, knees, paws)


class Sato4(leg_initializer):
    name = 'Sato4 - Fpi6M-pi6H-pi6'

    def __init__(self, parent):
        super().__init__(parent)
        knees = _rads_to_joint_position(0 * pi)
        paws = knees
        front_shoulders, _, _ = _shoulder_angle_calculation(pi / 6)
        _, mid_shoulders, hind_shoulders = _shoulder_angle_calculation(-pi / 6)
        self.update_init_angles(hind_shoulders, knees, paws,
                                mid_shoulders, knees, paws,
                                front_shoulders, knees, paws)


# ----------------------------------------------------------------------------------------------  Knees angle -pi/12 rad
class Sato5(Sato1):
    name = 'Sato5 - Sato1 with knee theta -pi12 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(-pi / 12)
        self.update_knees(knee_theta)


class Sato6(Sato2):
    name = 'Sato6 - Sato2 with knee theta -pi12 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(-pi / 12)
        self.update_knees(knee_theta)


class Sato7(Sato3):
    name = 'Sato7 - Sato3 with knee theta -pi12 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(-pi / 12)
        self.update_knees(knee_theta)


class Sato8(Sato4):
    name = 'Sato8 - Sato4 with knee theta -pi12 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(-pi / 12)
        self.update_knees(knee_theta)


# ------------------------------------------------------------------------------------------------  Knees angle pi/6 rad
class Sato9(Sato1):
    name = 'Sato9 - Sato1 with knee theta pi6 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(pi / 6)
        self.update_knees(knee_theta)


class Sato10(Sato2):
    name = 'Sato10 - Sato2 with knee theta pi6 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(pi / 6)
        self.update_knees(knee_theta)


class Sato11(Sato3):
    name = 'Sato11 - Sato3 with knee theta pi6 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(pi / 6)
        self.update_knees(knee_theta)


class Sato12(Sato4):
    name = 'Sato12 - Sato4 with knee theta pi6 rad'

    def __init__(self, parent):
        super().__init__(parent)
        knee_theta = _rads_to_joint_position(pi / 6)
        self.update_knees(knee_theta)


class OPT(leg_initializer):
    name = 'OPT - Fpi6M0H-pi6'

    def __init__(self, parent):
        super().__init__(parent)
        knees = _rads_to_joint_position(pi / 8)
        paws = _rads_to_joint_position(pi / 6)
        front_shoulders, _, _ = _shoulder_angle_calculation(pi / 6)
        _, mid_shoulders, _ = _shoulder_angle_calculation(0 * pi)
        _, _, hind_shoulders = _shoulder_angle_calculation(-pi / 6)
        self.update_init_angles(hind_shoulders, knees, paws,
                                mid_shoulders, knees, paws,
                                front_shoulders, knees, paws)


morphologies = [Sato1,
                Sato2,
                Sato3,
                Sato4,
                Sato5,
                Sato6,
                Sato7,
                Sato8,
                Sato9,
                Sato10,
                Sato11,
                Sato12]

if __name__ == '__main__':
    from hexapod_class import hexapod
    import control_system_class as cs
    import time

    hexapod.set_control_system(cs.v15)

    for morphology in morphologies:
        hexapod.set_init_config(morphology)
        hexapod.init_legs()
        time.sleep(5)

