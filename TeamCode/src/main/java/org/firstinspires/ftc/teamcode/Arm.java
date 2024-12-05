package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Arm", group="Linear OpMode")
// @Disabled
public class Arm extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public Servo grabber;
    public Servo tilt;
    public double tilt_position;
    public double tilt_add;
    public Servo arm;
    public double arm_position;
    private double i;

    //    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        // Set servo directions (keep default).
        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);


        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // Set the initial servo positions to neutral (mid-point).

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

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
                tilt.setPosition(0.5);
            }


            if (gamepad1.dpad_down) {
                arm.setPosition(0.9);
                tilt.setPosition(0);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position + 0.01);
            }

            while (gamepad1.left_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position - 0.01);
            }

            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}
