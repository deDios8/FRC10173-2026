import typing
import commands2
import commands2.cmd
from wpilib import SmartDashboard, Timer
from pathplannerlib.auto import AutoBuilder
from commands2.button import CommandXboxController
from subsystems.SS_GeneralMotor import SS_GeneralMotor
from subsystems.SS_GeneralServo import SS_GeneralServo
from subsystems.SS_EncodedMotor import SS_EncodedMotor
from subsystems.SS_SwerveDrive import SS_SwerveDrive



class RobotContainer:
    def __init__(self) -> None:
        self._joystick = CommandXboxController(0)
        self.initialize_subsystems()
        # self.initialize_swerve_drive()
        self.controller_bindings()


    def initialize_subsystems(self) -> None:
        self._joystick = commands2.button.CommandXboxController(0)
        self.ss_general_motor = SS_GeneralMotor()
        self.ss_general_servo = SS_GeneralServo()
        self.ss_encoded_motor = SS_EncodedMotor()


    def initialize_swerve_drive(self) -> None:
        self.ss_swerve_drive = SS_SwerveDrive(self._joystick)
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)


    def controller_bindings(self) -> None:
        # swerve drive bindings are contained in the SS_SwerveDrive class
        self._joystick.x().whileTrue(self.ss_general_motor.run_forward_command2())
        self._joystick.a().onFalse(self.ss_general_servo.run_to_min_position_command())
        self._joystick.b().onFalse(self.ss_general_servo.run_to_max_position_command())
        self._joystick.y().onFalse(self.ss_general_servo.run_to_A_position_command())
        
        self._joystick.rightBumper().whileTrue(self.ss_general_servo.adjust_servo_ahead_command())
        self._joystick.leftBumper().whileTrue(self.ss_general_servo.adjust_servo_reverse_command())

        self._joystick.povUp().whileTrue(self.ss_encoded_motor.run_forward_command())
        self._joystick.povDown().onTrue(self.ss_encoded_motor.stop_motor_command())
        self._joystick.povLeft().onTrue(self.ss_encoded_motor.go_to_destination_A_command())
        self._joystick.povRight().onTrue(self.ss_encoded_motor.go_to_destination_B_command())


    def getAutonomousCommand(self) -> commands2.Command:
        return self._auto_chooser.getSelected()



class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None: # Should this be __init__?
        self.container = RobotContainer()

    def robotPeriodic(self) -> None: # Called every 20 ms
        # TODO commands2.CommandScheduler.getInstance().run()
        SmartDashboard.putNumber("Match Time", Timer.getMatchTime());


    def autonomousInit(self) -> None:
        self.autonomousCommand = self.container.getAutonomousCommand()
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def teleopInit(self) -> None:
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def testInit(self) -> None:
        commands2.CommandScheduler.getInstance().cancelAll()



# TODO if __name__ == "__main__":
#     wpilib.run(MyRobot)