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
    deadzone = 0.05

    module_a_move_motor_id = 0
    module_a_rotation_motor_id = 1
    module_b_move_motor_id = 2
    module_b_rotation_motor_id = 3
    module_c_move_motor_id = 4
    module_c_rotation_motor_id = 5
    module_d_move_motor_id = 6
    module_d_rotation_motor_id = 7

    module_rotation_counts_per_revolution = 497.0*(40.0/48.0)
    module_rotation_volts_per_revolution = 3.3

    TAU = math.pi*2.0

    robot_width = 648.0 # mm
    robot_length = 386.394 # mm

    motor_dist = math.sqrt((robot_width/2)**2+(robot_length/2)**2) # distance of motors from the center of the robot

    #                    x component                   y component
    vz_components = ((robot_length/2) / motor_dist, (robot_width/2)/motor_dist) # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    motor_a_vz_scaling = (-1, 1)
    motor_b_vz_scaling = (-1, -1)
    motor_c_vz_scaling = (1, -1)
    motor_d_vz_scaling = (1, 1)

    module_angular_tol = 0.02 #approx 1 deg
