import random
import time

# Custom Modules
import control_system_class as cs
import morphology_class as mc
from hexapod_class import Constructor


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