package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop_final", group="Linear OpMode")
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
    public double leftpoddirection;
    public boolean rightpoddirection;
    public double left_theta, right_theta, prev_left_theta, prev_right_theta;
    public double left_magnitude, right_magnitude;
    public double left_forward, right_forward;

    public double magnitude_gain = 0.5;
    public float rightwheelposition;

    public Servo grabber;
    public Servo tilt;
    public double tilt_position;
    public Servo arm;
    public double arm_position;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        leftservo  = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);

        // Set servo directions (keep default).
        leftservo.setDirection(Servo.Direction.FORWARD);
        rightservo.setDirection(Servo.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).
        leftservo.setPosition(0.5);
        rightservo.setPosition(0.5);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
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
            if (left_magnitude > 0.2 && (prev_left_theta < left_theta + 0.2 || prev_left_theta > left_theta - 0.2)) {
                left_theta = prev_left_theta;
                leftjoystickactive = true;
            } else {
                prev_left_theta = left_theta;
            }

            telemetry.addData("Left stick x", leftStickX);
            telemetry.addData("Left stick y", leftStickY);
            telemetry.addData("Right Stick x", rightStickX);
            telemetry.addData("Right Stick y", rightStickY);
            telemetry.addData("Left stick theta", left_theta);
            telemetry.addData("Right Stick theta", right_theta);
            telemetry.addData("Left stick mag", left_magnitude);
            telemetry.addData("Right Stick mag", right_magnitude);

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
            }


            // independent
            // rightwheel.setPower(right_forward*right_magnitude * magnitude_gain);


            telemetry.addData("Left Servo Position", leftservo.getPosition());
            telemetry.addData("Right Servo Position", rightservo.getPosition());
            telemetry.update();
            idle();

            // Servo control with deadzone



            while (gamepad1.dpad_left) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position + 0.01);
            }

            while (gamepad1.dpad_right) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position - 0.01);
            }

            if (gamepad1.y) {
                tilt.setPosition(0.5);
            }
//
            if (gamepad1.a) {
                tilt.setPosition(0);
            }

            if (gamepad1.x) {
                grabber.setPosition(0);
            }

            if (gamepad1.b) {
                grabber.setPosition(1);
            }

            if (gamepad1.dpad_up) {
                arm.setPosition(0);
            }

            if (gamepad1.dpad_down) {
                arm.setPosition(0.9);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position + 0.01);
            }

            while (gamepad1.left_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position - 0.01);
            }


            // Send calculated power to wheels

            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Servo Position", leftservo.getPosition());
            telemetry.addData("Right Servo Position", rightservo.getPosition());
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("Left Stick X", leftStickX);
            telemetry.addData("Right Stick X", rightStickX);
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}
