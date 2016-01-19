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

    module_b_move_motor_id = 4 #put in
    module_b_rotation_motor_id = 7 # put in
    module_c_move_motor_id = 2 # put in
    module_c_rotation_motor_id = 6 # put in
    module_d_move_motor_id = 1 # put in
    module_d_rotation_motor_id = 5 # put in
    module_a_move_motor_id = 3 # put in
    module_a_rotation_motor_id = 8 # put in

    module_rotation_counts_per_revolution = 497.0*(40.0/48.0) * 4.0
    module_rotation_volts_per_revolution = 3.3

    TAU = math.pi*2.0

    robot_length= 648.0 # mm
    robot_width = 386.394 # mm

    motor_dist = math.sqrt((robot_width/2)**2+(robot_length/2)**2) # distance of motors from the center of the robot

    #                    x component                   y component
    vz_sensitivity = 1.0
    vz_components = ((robot_width/2) / motor_dist * vz_sensitivity, (robot_length/2)/motor_dist * vz_sensitivity) # multiply both by vz and the

    # the number that you need to multiply the vz components by to get them in the appropriate directions
    #                   vx   vy
    motor_vz_scaling = [(-vz_components[0], vz_components[1]),
                        (-vz_components[0], -vz_components[1]),
                        (vz_components[0], -vz_components[1]),
                        (vz_components[0], vz_components[1])]

    module_angular_tol = 0.02 #approx 1 deg

    steering_p = 6.0

    move_forward_seconds = 3.0 # seconds
    move_forward_seconds_button = 11
