package useless;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Right", preselectTeleOp = "Gamepad")
@Disabled
public class AutoRight extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        setupAndWait();
        Servo leftservo, rightservo;
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");

        rightservo.setPosition(1);
        leftservo.setPosition(1);
        sleep(100);
        robotDriver.gyroDrive(0.6d, 3d, 0d, 5d, null);
        sleep(100);
        rightservo.setPosition(0.5);
        leftservo.setPosition(0.5);
        sleep(100);
        robotDriver.gyroDrive(0.6d, -10d, 0d, 5d, null);
    }

    @TeleOp(name="PIDSwerve_drive", group="Linear OpMode")
     @Disabled
    public static class PID_swerve extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        public DcMotor leftwheel = null;
        public DcMotor rightwheel = null;
        public Servo leftservo;
        public Servo rightservo;
        public double rightpodposition;
        public double leftpodposition;
        public double leftpoddirection;
        public boolean rightpoddirection;
        public double left_theta, right_theta, prev_left_theta, prev_right_theta;
        public double left_magnitude, right_magnitude;
        public double left_forward, right_forward;
        public double leftMotorPower, rightMotorPower;
        public double leftServoTarget, rightServoTarget;
        public double leftServoPower, rightServoPower;
        public boolean moving;

        public double magnitude_gain = 0.5;
        public float rightwheelposition;

        // Declare PID controllers for motors and servos
        private PIDController leftMotorPID = new PIDController(0.1, 0.0, 0.0); // PID constants for left motor
        private PIDController rightMotorPID = new PIDController(0.1, 0.0, 0.0); // PID constants for right motor
        private PIDController leftServoPID = new PIDController(0.05, 0.0, 0.0); // PID constants for left servo
        private PIDController rightServoPID = new PIDController(0.05, 0.0, 0.0); // PID constants for right servo

        @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            leftwheel = hardwareMap.get(DcMotor.class, "leftwheel");
            rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
            leftservo = hardwareMap.get(Servo.class, "leftservo");
            rightservo = hardwareMap.get(Servo.class, "rightservo");
            leftwheel.setDirection(DcMotor.Direction.REVERSE);
            rightwheel.setDirection(DcMotor.Direction.FORWARD);
            leftservo.setDirection(Servo.Direction.FORWARD);
            rightservo.setDirection(Servo.Direction.FORWARD);

            waitForStart();
            runtime.reset();

            leftservo.setPosition(0.5);
            rightservo.setPosition(0.5);
            moving = false;

            while (opModeIsActive()) {

                boolean leftMotorForward = true;
                boolean rightMotorForward = true;
                boolean leftjoystickactive = false;
                double leftPower;
                double rightPower;
                double radius;
                radius = 1;
                rightpodposition = 1;
                leftpodposition = 1;

                // Apply deadzone for joystick X-axis
                double leftStickX = gamepad1.left_stick_x;
                double rightStickX = gamepad1.right_stick_x;
                double leftStickY = gamepad1.left_stick_y;
                double rightStickY = gamepad1.right_stick_y;
                double deadzone = 0;

                // theta = direction), magnitude = speed
                left_theta = -Math.atan2(leftStickY, leftStickX);
                right_theta = -Math.atan2(rightStickY, rightStickX);

                left_magnitude = Math.sqrt(Math.pow(leftStickY, 2.0) + Math.pow(leftStickX, 2.0));
                right_magnitude = Math.sqrt(Math.pow(rightStickY, 2.0) + Math.pow(rightStickX, 2.0));

                // avoid singular point for servo position.
                if (left_magnitude > 0.2 && ((prev_left_theta < (left_theta + 0.2)) || (prev_left_theta > (left_theta - 0.2)))) {
                    left_theta = prev_left_theta;
                    leftjoystickactive = true;
                } else {
                    prev_left_theta = left_theta;
                }

    //            telemetry.addData("Left stick x", leftStickX);
    //            telemetry.addData("Left stick y", leftStickY);
    //            telemetry.addData("Right Stick x", rightStickX);
    //            telemetry.addData("Right Stick y", rightStickY);
    //            telemetry.addData("Left stick theta", left_theta);
    //            telemetry.addData("Right Stick theta", right_theta);
    //            telemetry.addData("Left stick mag", left_magnitude);
    //            telemetry.addData("Right Stick mag", right_magnitude);

                left_forward = 1.0;
                right_forward = 1.0;

                if (leftjoystickactive) {
                    if (left_theta < 0) {
                        left_forward = -1.0;
                        left_theta = left_theta * -1;
                    } else {
                        left_theta = Math.PI - left_theta;
                    }
                    if (right_theta < 0) {
                        right_forward = -1.0;
                        right_theta = right_theta * -1;
                    }

                    // PID for servo positions
                    leftServoTarget = left_theta / Math.PI;
                    rightServoTarget = left_theta / Math.PI;

                    leftServoPower = leftServoPID.calculate(leftservo.getPosition(), leftServoTarget);
                    rightServoPower = rightServoPID.calculate(rightservo.getPosition(), rightServoTarget);

                    leftservo.setPosition(leftServoPower); // Apply PID output to servo
                    rightservo.setPosition(rightServoPower);

                    leftMotorPower = leftMotorPID.calculate(leftwheel.getCurrentPosition(), left_forward * left_magnitude * magnitude_gain); // target
                    rightMotorPower = rightMotorPID.calculate(rightwheel.getCurrentPosition(), left_forward * left_magnitude * magnitude_gain);
                    leftwheel.setPower(leftMotorPower);  // Apply PID-adjusted motor power
                    rightwheel.setPower(rightMotorPower);  // Apply PID-adjusted motor power
                } else if (right_magnitude > 0.2 && !leftjoystickactive) {
                    magnitude_gain = 0.5;
                    leftservo.setPosition(0.5);
                    rightservo.setPosition(0.5);
                    leftwheel.setPower(rightStickX * magnitude_gain);
                    rightwheel.setPower(rightStickX * -1 * magnitude_gain);
                } else {
                    leftwheel.setPower(0);
                    rightwheel.setPower(0);

                    if (gamepad1.dpad_up) {
                        if (!moving) {
                            leftservo.setPosition(0.5);
                            rightservo.setPosition(0.5);
                            leftwheel.setPower(0.1);
                            rightwheel.setPower(0.1);
                        } else {
                            leftwheel.setPower(0);
                            rightwheel.setPower(0);
                        }
                    }

                    if (gamepad1.dpad_down) {
                        if (!moving) {
                            leftservo.setPosition(0.5);
                            rightservo.setPosition(0.5);
                            leftwheel.setPower(-0.1);
                            rightwheel.setPower(-0.1);
                        } else {
                            leftwheel.setPower(0);
                            rightwheel.setPower(0);
                        }
                    }

                    if (gamepad1.dpad_left) {
                        if (!moving) {
                            leftservo.setPosition(0);
                            rightservo.setPosition(0);
                            leftwheel.setPower(0.1);
                            rightwheel.setPower(0.1);
                        } else {
                            leftwheel.setPower(0);
                            rightwheel.setPower(0);
                        }
                    }

                    if (gamepad1.dpad_right) {
                        if (!moving) {
                            leftservo.setPosition(1);
                            rightservo.setPosition(1);
                            leftwheel.setPower(0.1);
                            rightwheel.setPower(0.1);
                        } else {
                            leftwheel.setPower(0);
                            rightwheel.setPower(0);
                        }
                    }



                }

                // telemetry
                telemetry.addData("Left Servo Position", leftservo.getPosition());
                telemetry.addData("Right Servo Position", rightservo.getPosition());
                telemetry.addData("Left Servo target position", leftServoTarget);
                telemetry.addData("Right Servo target position", rightServoTarget);
                telemetry.addData("Left Servo power", leftServoPower);
                telemetry.addData("Right Servo power", rightServoPower);
                telemetry.addData("-----------------------------", 0);
                telemetry.addData("Left wheel power", leftwheel.getPower());
                telemetry.addData("Right wheel power", rightwheel.getPower());
                telemetry.addData("Left wheel target power", leftMotorPower);
                telemetry.addData("Right wheel target power", rightMotorPower);
                telemetry.update();
                idle();
            }
        }
    }
}