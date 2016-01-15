import wpilib
from wpilib import buttons

from robot_map import RobotMap
"""
ROBOT AXIS we use these axis mappings, which are different from the wpilib ones
      + X
        ^
        |
+ Y <------->
        |
        V
Z axis pointing up, with a +ve rotation beting counter clockwise (right hand rule)

WPILIB AXIS MAPPINGS - used when getting axis from a joystick or gamepad

        ^
        |
    <------> + X
        |
        V
        +Y
"""

class OI:
    """
    This class is the glue that binds the controls on the physical operator
    interface to the example_command and command groups that allow control of the robot.
    """

    def __init__(self, robot):
        self.robot = robot

        self.joystick = wpilib.Joystick(RobotMap.joystick_port)
        self.gamepad = wpilib.Joystick(RobotMap.gamepad_port)

        #CREATING BUTTONS
        #One type of button is a joystick button which is any button on a joystick.
        #You create one by telling it which joystick it's on and which button
        #number it is.
        #stick = wpilib.Joystick(port)
        #button = buttons.JoystickButton(stick, button_number)

        #There are a few additional built-in buttons you can use. Additionally, by
        #subclassing Button you can create custom example_trigger and bind those to
        #example_command the same as any other Button

        #TRIGGERING COMMANDS WITH BUTTONS
        #Once you have a button, it's trivial to bind it to a button in one of
        #three ways;

        #Start the command when the button is pressed and let it run the command
        #until it is finished as determined by it's isFinished method.
        #button.whenPressed(ExampleCommand())

        #Run the command while the button is being held down and interrupt it
        #once the button is released
        #button.whileHeld(ExampleCommand())

        #Start the command when the button is released and let it run the command
        #until it is finished as determined by it's isFinished method.
        #button.whenReleased(ExampleCommand())

    def getLeftStickX(self): #get the x axis of the left side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_x)
        axis=self.applyDeadzone(axis)
        return axis


    def getLeftStickY(self): #get the y axis of the left side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_y)
        axis=self.applyDeadzone(axis)
        return axis

    def getRightStickX(self): #get the x axis of the right side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_right_stick_x)
        axis=self.applyDeadzone(axis)
        return axis

    def getRightStickY(self): #get the y axis of the right side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_y)
        axis=self.applyDeadzone(axis)
        return axis

    def applyDeadzone(self, axis, deadzone):
        if abs(axis)< deadzone:
            return 0.0
        return axis

    def getJoystickX(self):
        axis = self.joystick.getX()
        axis = self.applyDeadzone(axis, RobotMap.joystick_x_deadzone)
        return axis

    def getJoystickY(self):
        axis = self.joystick.getY()
        axis = self.applyDeadzone(axis, RobotMap.joystick_y_deadzone)
        return axis

    def getJoystickZ(self):
        axis = self.joystick.getZ()
        axis = self.applyDeadzone(axis, RobotMap.joystick_z_deadzone)
        return axis

    def getThrottle(self):
        return (self.joystick.getThrottle()-1.0)/-2.0
