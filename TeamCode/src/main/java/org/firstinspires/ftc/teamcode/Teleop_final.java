package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@TeleOp(name="Teleop final", group="Linear OpMode")
// @Disabled
public class Teleop_final extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo leftservo;
    public Servo rightservo;
    public double rightpodposition;
    public double leftpodposition;
    public double left_theta, right_theta, prev_left_theta, prev_right_theta;
    public double left_magnitude, right_magnitude;
    public double left_forward, right_forward;
    public boolean moving;

    public Servo grabber;
    public Servo tilt;
    public Servo arm;
    private boolean grabbing;
//    public Servo slide;
    public DcMotor intake;
    public double slide_position;
    public boolean taking;

    public double magnitude_gain = 0.5;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");
//        slide = hardwareMap.get(Servo.class, "slide");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);

        // Set servo directions (keep default).
        leftservo.setDirection(Servo.Direction.FORWARD);
        rightservo.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).
        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);
        moving = false;
        taking = false;
        grabbing = false;

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
                    left_theta = Math.PI - left_theta;
                }
                if (right_theta < 0) {
                    right_forward = -1.0;
                    right_theta = right_theta * -1;
                }
                leftservo.setPosition(left_theta / Math.PI);
                rightservo.setPosition(left_theta / Math.PI);

                leftwheel.setPower(left_forward * left_magnitude * magnitude_gain);
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
                        leftwheel.setPower(0.6);
                        rightwheel.setPower(0.6);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_down) {
                    if (!moving) {
                        leftservo.setPosition(0.5);
                        rightservo.setPosition(0.5);
                        leftwheel.setPower(-0.6);
                        rightwheel.setPower(-0.6);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_left) {
                    if (!moving) {
                        leftservo.setPosition(0);
                        rightservo.setPosition(0);
                        leftwheel.setPower(0.6);
                        rightwheel.setPower(0.6);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }

                if (gamepad1.dpad_right) {
                    if (!moving) {
                        leftservo.setPosition(1);
                        rightservo.setPosition(1);
                        leftwheel.setPower(0.6);
                        rightwheel.setPower(0.6);
                    } else {
                        leftwheel.setPower(0);
                        rightwheel.setPower(0);
                    }
                }
            }


            if (gamepad1.x) {
                tilt.setPosition(0.4);
                sleep(100);
                arm.setPosition(0.5);
                sleep(100);
                tilt.setPosition(0.7);
                arm.setPosition(0);
            }

            if (gamepad1.y) {
                arm.setPosition(0);
                tilt.setPosition(0.1);
            }


            if (gamepad1.b) {
                if (!grabbing) {
                    grabber.setPosition(0.5);
                    grabbing = true;
                } else {
                    grabber.setPosition(0);
                    grabbing = false;
                }
            }

            if (gamepad1.a) {
                if (!taking) {
                    intake.setPower(1);
                    taking = true;
                } else {
                    intake.setPower(0);
                    taking = false;
                }
            }


            telemetry.addData("Left Servo Position", leftservo.getPosition());
            telemetry.addData("Right Servo Position", rightservo.getPosition());
            telemetry.addData("moving?", moving );
            telemetry.addData("-------------------------------", null);
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("arm Servo Position", arm.getPosition());
//            telemetry.addData("Linear Slide Position", slide.getPosition());
            telemetry.addData("intaking?", taking);
            telemetry.update();
            idle();

//
        }

        // Telemetry to display key data


    }

}
