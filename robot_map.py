
class RobotMap:
    """
    The RobotMap is a mapping from the ports sensors and actuators are wired into
    to a variable name. This provides flexibility changing wiring, makes checking
    the wiring easier and significantly reduces the number of magic numbers
    floating around.
    """

    # For example to map the left and right motors, you could define the
    # following variables to use with your drivetrain subsystem.
    # left_motor = 1
    # right_motor = 2
    gamepad_port = 0
    deadzone = 0.05

    module_a_move_motor_id = 0
    module_a_rotation_motor_id = 1
    module_b_move_motor_id = 2
    module_b_rotation_motor_id = 3
    module_c_move_motor_id = 4
    module_c_rotation_motor_id = 5
    module_d_move_motor_id = 6
    module_d_rotation_motor_id = 7
