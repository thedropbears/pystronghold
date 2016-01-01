from wpilib.command import Command, CommandGroup


#This is a template Command, ExampleCommand should, of course, be replaced by the name of your desired Command
class ExampleCommand(Command):

    def __init__(self, robot, name=None, timeout=None):
        """This is the constructor of the command, use this to declare subsystem dependencies"""
        #Use self.requires() here to declare subsystem dependencies
        #eg. self.requires(chassis)
        super().__init__(name, timeout)
        self.requires(robot.example_subsystem)
        self.robot = robot

    def initialize(self):
        """Called just before this Command runs the first time"""
        pass

    def execute(self):
        """Called repeatedly when this Command is scheduled to run"""
        pass

    def isFinished(self):
        """This should return true when this command no longer needs to run execute()"""
        return False

    def end(self):
        """Called once after isFinished returns true"""
        pass

    def interrupted(self):
        """Called when another command which requires one or more of the same example_subsystem is scheduled to run"""
        pass


class ExampleCommandGroup(CommandGroup):
    """
    This is a template Command Group, CommandGroupName should, of course, be replaced by the
    name of your desired Command Group
    """

    def __init__(self, robot, name=None):
        # Add Commands here:
        # e.g. addSequential(Command1(robot))
        #      addSequential(Command2(robot))
        # these will run in order.

        # To run multiple commands at the same time,
        # use addParallel()
        # e.g. addParallel(Command1(robot))
        #      addSequential(Command2(robot))
        # Command1 and Command2 will run in parallel.

        # A command group will require all of the example_subsystem that each member
        # would require.
        # e.g. if Command1 requires chassis, and Command2 requires arm,
        # a CommandGroup containing them would require both the chassis and the
        # arm.
        super().__init__(name)
        self.robot = robot
