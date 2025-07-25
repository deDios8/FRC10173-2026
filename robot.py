import typing
import commands2
import commands2.cmd
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from generated.tuner_constants import TunerConstants
from telemetry import Telemetry
from pathplannerlib.auto import AutoBuilder
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Rotation2d
from wpimath.units import rotationsToRadians
from subsystems.SS_GeneralMotor import SS_GeneralMotor
from subsystems.SS_GeneralServo import SS_GeneralServo
from subsystems.SS_EncodedMotor import SS_EncodedMotor



class RobotContainer:
    def __init__(self) -> None:
        self._logger = Telemetry(self._max_speed)
        self._auto_chooser = AutoBuilder.buildAutoChooser("Tests")
        SmartDashboard.putData("Auto Mode", self._auto_chooser)

        self._joystick = CommandXboxController(0)
        self.setup_swerve_drivetrain()
        self.swerve_bindings()
        self.controller_bindings()

    def initialize_subsystems(self):
        self.controller = commands2.button.CommandXboxController(0)
        self.ss_general_motor = SS_GeneralMotor()
        self.ss_general_servo = SS_GeneralServo()
        self.ss_encoded_motor = SS_EncodedMotor()

    def controller_bindings(self) -> None:
        self.controller.x().whileTrue(self.ss_general_motor.run_forward_command2())
        self.controller.a().onFalse(self.ss_general_servo.run_to_min_position_command())
        self.controller.b().onFalse(self.ss_general_servo.run_to_max_position_command())
        self.controller.y().onFalse(self.ss_general_servo.run_to_A_position_command())
        
        self.controller.rightBumper().whileTrue(self.ss_general_servo.adjust_servo_ahead_command())
        self.controller.leftBumper().whileTrue(self.ss_general_servo.adjust_servo_reverse_command())

        self.controller.povUp().whileTrue(self.ss_encoded_motor.run_forward_command())
        self.controller.povDown().onTrue(self.ss_encoded_motor.stop_motor_command())
        self.controller.povLeft().onTrue(self.ss_encoded_motor.go_to_destination_A_command())
        self.controller.povRight().onTrue(self.ss_encoded_motor.go_to_destination_B_command())


    def swerve_bindings(self) -> None:
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.drivetrain.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.drivetrain.apply_request( lambda: (
                self._drive_field_centered
                    .with_velocity_x( # Drive forward with negative Y (forward)
                        -self._joystick.getLeftY() * self._max_speed )  
                    .with_velocity_y( # Drive left with negative X (left)
                        -self._joystick.getLeftX() * self._max_speed )  
                    .with_rotational_rate( # Drive counterclockwise with negative X (left)
                        -self._joystick.getRightX() * self._max_angular_rate ) ) ) )
        
        pov_speed = 0.2
        (self._joystick.start() and self._joystick.pov(0)).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._drive_robot_centered.with_velocity_x(pov_speed).with_velocity_y(0) ) )
        (self._joystick.start() and self._joystick.pov(180)).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._drive_robot_centered.with_velocity_x(-pov_speed).with_velocity_y(0) ) )
        (self._joystick.start() and self._joystick.pov(90)).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._drive_robot_centered.with_velocity_x(0).with_velocity_y(pov_speed) ) )
        (self._joystick.start() and self._joystick.pov(270)).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._drive_robot_centered.with_velocity_x(0).with_velocity_y(-pov_speed) ) )

        (self._joystick.back() & self._joystick.start()).onTrue( # reset the field-centric heading on left bumper press
            self.drivetrain.runOnce(lambda: self.drivetrain.seed_field_centric() ) )

        self.drivetrain.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        # Idle while the robot is disabled. This ensures the configured
        idle = swerve.requests.Idle() 
        Trigger(DriverStation.isDisabled).whileTrue(
            self.drivetrain.apply_request(lambda: idle).ignoringDisable(True) )
        # Brake the drivetrain when back is pressed with and B is pressed.
        (self._joystick.back() & self._joystick.b()).whileTrue(
            self.drivetrain.apply_request(lambda: self._brake) )
        # Point the wheels at the joystick direction when back and A are pressed
        (self._joystick.back() & self._joystick.b()).whileTrue(
            self.drivetrain.apply_request(
                lambda: self._point_wheels_at_direction.with_module_direction(
                    Rotation2d(-self._joystick.getLeftY(), -self._joystick.getLeftX()) ) ) )
        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self._joystick.start() & self._joystick.a()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.b()).whileTrue(
            self.drivetrain.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self._joystick.start() & self._joystick.y()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self._joystick.start() & self._joystick.x()).whileTrue(
            self.drivetrain.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

    def setup_swerve_drivetrain(self) -> None:
        self._max_speed = (TunerConstants.speed_at_12_volts)
        self._max_angular_rate = rotationsToRadians(0.75)

        # Setting up bindings for necessary control of the swerve drive platform
        self._drive_field_centered = (
            swerve.requests.FieldCentric()
            .with_deadband(self._max_speed * 0.1)
            .with_rotational_deadband(self._max_angular_rate * 0.1)  # Add a 10% deadband
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE) )
        self._drive_robot_centered = (
            swerve.requests.RobotCentric()
            .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE ) )
        self._brake = swerve.requests.SwerveDriveBrake()
        self._point_wheels_at_direction = swerve.requests.PointWheelsAt()
        self.drivetrain = TunerConstants.create_drivetrain()

    def getAutonomousCommand(self) -> commands2.Command:
        return self._auto_chooser.getSelected()



class MyRobot(commands2.TimedCommandRobot):
    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None: # Should this be __init__?
        self.container = RobotContainer()

    def robotPeriodic(self) -> None: # Called every 20 ms
        # TODO commands2.CommandScheduler.getInstance().run()
        pass

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