package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Swerve_drive", group="Linear OpMode")
// @Disabled
public class Swerve_drive extends LinearOpMode {
    private final Robot Robot = new Robot();

    //    @Override
    public void runOpMode() {
        robot.init(hardwareMap)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).
        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);
        moving = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
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


            left_theta = -Math.atan2(leftStickY, leftStickX);
            right_theta = -Math.atan2(rightStickY, rightStickX);


            left_magnitude = Math.sqrt(Math.pow(leftStickY, 2.0) + Math.pow(leftStickX, 2.0));
            right_magnitude = Math.sqrt(Math.pow(rightStickY, 2.0) + Math.pow(rightStickX, 2.0));

            // avoid singular point for serve position.
            if (left_magnitude > 0.2 && ((prev_left_theta < (left_theta + 0.2)) || (prev_left_theta > (left_theta - 0.2)))) {
                left_theta = prev_left_theta;
                leftjoystickactive = true;
            } else {
                prev_left_theta = left_theta;
            }

            left_forward = 1.0;
            right_forward = 1.0;
            if (leftjoystickactive) {
                if (left_theta < 0) {
                    left_forward = -1.0;
                    left_theta = left_theta * -1;
                } else {
                    // left_theta = (left_theta - Math.PI/2) * -1 + Math.PI/2 ;
                    left_theta = Math.PI - left_theta;
                }
                if (right_theta < 0) {
                    right_forward = -1.0;
                    right_theta = right_theta * -1;
                }
                leftservo.setPosition(left_theta / Math.PI);
                // synchronzied with left joystick
                rightservo.setPosition(left_theta / Math.PI);
                // independent
                // rightservo.setPosition(right_theta / Math.PI);

                leftwheel.setPower(left_forward * left_magnitude * magnitude_gain);
                // synchronzied with left joystick
                rightwheel.setPower(left_forward * left_magnitude * magnitude_gain);
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
                        leftwheel.setPower(0.4);
                        rightwheel.setPower(0.4);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_down) {
                    if (!moving) {
                        leftservo.setPosition(0.5);
                        rightservo.setPosition(0.5);
                        leftwheel.setPower(-0.4);
                        rightwheel.setPower(-0.4);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_left) {
                    if (!moving) {
                        leftservo.setPosition(0);
                        rightservo.setPosition(0);
                        leftwheel.setPower(0.4);
                        rightwheel.setPower(0.4);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_right) {
                    if (!moving) {
                        leftservo.setPosition(1);
                        rightservo.setPosition(1);
                        leftwheel.setPower(0.4);
                        rightwheel.setPower(0.4);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }
            }


            // independent
            // rightwheel.setPower(right_forward*right_magnitude * magnitude_gain);


            telemetry.addData("Left Servo Position", leftservo.getPosition());
            telemetry.addData("Right Servo Position", rightservo.getPosition());
            telemetry.addData("moving?", moving );
            telemetry.update();
            idle();

//
        }

        // Telemetry to display key data


    }

}
