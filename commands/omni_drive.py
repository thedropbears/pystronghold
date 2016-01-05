from wpilib.command import Command, CommandGroup

class TankDrive(Command):

    def __init__(self, robot, name=None, timeout=None):
        """This is the constructor of the command, use this to declare subsystem dependencies"""
        #Use self.requires() here to declare subsystem dependencies
        #eg. self.requires(chassis)
        super().__init__(name, timeout)
        self.requires(robot.chassis)
        self.robot = robot

    def initialize(self):
        """Called just before this Command runs the first time"""
        robot.drive(0.0, 0.0, 0.0, 0.0) # just to be safe

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        robot.chassis.drive(robot.oi.getLeftStickX(), robot.oi.getLeftStickY(), robot.oi.getRightStickY(), 1.0)

    def isFinished(self):
        """This should return true when this command no longer needs to run execute()"""
        return False

    def end(self):
        """Called once after isFinished returns true"""
        robot.chassis.drive(0.0, 0.0, 0.0, 0.0)

    def interrupted(self):
        """Called when another command which requires one or more of the same example_subsystem is scheduled to run"""
        self.end()
