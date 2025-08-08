import wpilib
import constants
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

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        RobotContainer()
        self.joystick = wpilib.Joystick(1)  # Initialize joystick on port 1(engineer)
        

    def teleopPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

class RobotContainer:
    def __init__(self) -> None:
        # self.driver_joystick = CommandXboxController(0)
        self.initialize_subsystems()
        self.initialize_swerve_drive()
        self.driver_controller_bindings()
        self.engineer_controller_bindings
        RobotContainer()


    def initialize_subsystems(self) -> None:
        self.driver_joystick = commands2.button.CommandXboxController(0)
        self.engineer_joystick = commands2.button.CommandXboxController(1)
        self.ss_general_motor = SS_GeneralMotor()
        self.ss_encoded_motor = SS_EncodedMotor()
        self.ss_winch_servo = SS_GeneralServo(constants.PWM_CHANNELS["WINCH_SERVO"],
                                              constants.DASHBOARD_TITLES["WINCH_SERVO_POSITION"],
                                              pw_min=600, pw_center=1500, pw_max=2400) # HS-785HB
        self.ss_180_servo = SS_GeneralServo(constants.PWM_CHANNELS["180_SERVO"],
                                            constants.DASHBOARD_TITLES["180_SERVO_POSITION"],
                                            pw_min=1000, pw_center=1500, pw_max=2000,
                                            pos_min=0, pos_max=1) # SG-5010


    def initialize_swerve_drive(self) -> None:
        self.ss_swerve_drive = SS_SwerveDrive(self.driver_joystick)
        self._auto_chooser = AutoBuilder.buildAutoChooser(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"])
        SmartDashboard.putData(constants.SWERVE_DEFAULT_NOT_GENERATED["DEFAULT_AUTONOMOUS"], self._auto_chooser)


    def driver_controller_bindings(self) -> None:
        # swerve drive bindings are contained in the SS_SwerveDrive class
        #self.driver_joystick.x().whileTrue(self.ss_general_motor.run_forward_command2())#not anymore
        #self.driver_joystick.y().whileTrue(self.ss_general_motor.run_reverse_command2()) #not anymore
        # self.driver_joystick.a().onFalse(self.ss_180_servo.run_to_min_position_command())
        # self.driver_joystick.b().onFalse(self.ss_180_servo.run_to_max_position_command())
        # self.driver_joystick.y().onFalse(self.ss_180_servo.run_to_A_position_command())
        
        self.driver_joystick.rightBumper().whileTrue(self.ss_winch_servo.adjust_servo_ahead_command())
        self.driver_joystick.leftBumper().whileTrue(self.ss_winch_servo.adjust_servo_reverse_command())

        # self.driver_joystick.povUp().whileTrue(self.ss_encoded_motor.run_forward_command())
        self.driver_joystick.povUp().onTrue(self.ss_encoded_motor.go_to_destination_B_command())
        self.driver_joystick.povDown().onTrue(self.ss_encoded_motor.go_to_destination_A_command())
        self.driver_joystick.povLeft().onTrue(self.ss_encoded_motor.stop_motor_command())

    def engineer_controller_bindings(self) -> None:
        self.engineer_joystick.rightBumper().whileTrue(self.ss_general_motor.run_forward_command2())
        self.engineer_joystick.leftBumper().whileTrue(self.ss_general_motor.run_reverse_command2())




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