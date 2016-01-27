import math

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

    joystick_port = 0
    gamepad_port = 1
    joystick_x_deadzone = 0.05
    joystick_y_deadzone = 0.05
    joystick_z_deadzone = 0.4

    drive_motors_motor_a_id = 2
    drive_motors_motor_b_id = 5

    module_angular_tol = 0.02  # approx 1 deg

    range_finder_dio_channel = 0
