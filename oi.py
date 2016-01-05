import wpilib
from wpilib import buttons

from robot_map import RobotMap
"""
AXIS MAPPINGS we use these axis mappings, which are different from the wpilib ones
      + Y
        ^
        |
+ X <---|--->
        |
        V
"""

class OI:
    """
    This class is the glue that binds the controls on the physical operator
    interface to the example_command and command groups that allow control of the robot.
    """
    
    def __init__(self, robot):
        self.robot = robot

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

    def getLeftStickX(): #get the x axis of the left side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_y) # get the x axis of the left stick, which is considered by wpilib to be the y axis
        axis = -axis # reverse the direciton of the axis to make it consistant with our mappings
        if abs(axis) < deadzone:
            axis = 0.0
        return axis


    def getLeftStickY(): #get the y axis of the left side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_x) # get the y axis of the left stick, which is considered by wpilib to be the x axis
        axis = -axis # reverse the direciton of the axis to make it consistant with our mappings
        if abs(axis) < deadzone:
            axis = 0.0
        return axis

    def getRightStickX(): #get the x axis of the right side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_right_stick_y) # get the x axis of the right stick, which is considered by wpilib to be the y axis
        axis = -axis # reverse the direciton of the axis to make it consistant with our mappings
        if abs(axis) < deadzone:
            axis = 0.0
        return axis

    def getRightStickY(): #get the y axis of the right side of the joystick
        axis = self.gamepad.getAxis(RobotMap.gamepad_left_stick_x) # get the y axis of the right stick, which is considered by wpilib to be the x axis
        axis = -axis # reverse the direciton of the axis to make it consistant with our mappings
        if abs(axis) < deadzone:
            axis = 0.0
        return axis
