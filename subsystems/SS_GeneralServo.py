import wpilib
import commands2
import constants
import wpimath.units


class SS_GeneralServo(commands2.Subsystem):
    def __init__(self, pwm_channel, dashboard_position_text,
                 pw_min=1000, pw_center=1500, pw_max=2000, deadband=8,
                 pos_min=0.4, pos_max=0.6, pos_A=0.5):
        super().__init__()

        self.servo = wpilib.Servo(pwm_channel)
        self.dashboard_position_text = dashboard_position_text
        pulse_width_min = wpimath.units.microseconds(pw_min)
        pulse_width_center = wpimath.units.microseconds(pw_center)
        pulse_width_max = wpimath.units.microseconds(pw_max)
        deadband = wpimath.units.microseconds(8)
        self.servo.setBounds(pulse_width_min, deadband, pulse_width_center, deadband, pulse_width_max)
        self.run_speed = 0.001
        self.min_position = pos_min
        self.max_position = pos_max
        self.position_A = pos_A
        self.set_destination(self.position_A)
        self.joystick = wpilib.Joystick(1)  # Initialize joystick on port 1(engineer)

    def periodic(self): # Special function called periodically by the robot
        wpilib.SmartDashboard.putNumber(self.dashboard_position_text, self.position)
        speed = self.joystick.getY()  # Get the Y-axis value from the joystick
        self.servo.set(speed)
        direction = self.joystick.getX()  # Get the X-axis value from the joystick
        self.servo.set(-direction)  # Set the motor speed based on joystick input
        

    ## Methods
    def set_destination(self, destination):
        self.position = destination
        self.servo.set(self.position)

    def adjust_position(self, direction):
        this_run_speed = self.run_speed * direction
        new_position_capped_at_min = min(self.position + this_run_speed, self.max_position)
        new_position_within_caps = max(new_position_capped_at_min, self.min_position)
        self.set_destination(new_position_within_caps)


    ## Commands
    def run_to_min_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.min_position), self)

    def run_to_max_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.max_position), self)

    def run_to_A_position_command(self):
        return commands2.cmd.runOnce(lambda: self.set_destination(self.position_A), self)

    def adjust_servo_ahead_command(self):
        return commands2.cmd.run(lambda: self.adjust_position(1), self)

    def adjust_servo_reverse_command(self):
        return commands2.cmd.run(lambda: self.adjust_position(-1), self)

