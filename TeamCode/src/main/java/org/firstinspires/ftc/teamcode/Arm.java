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

            if (gamepad1.y) {
                tilt.setPosition(0.5);
            }
//
            if (gamepad1.a) {
                tilt.setPosition(0.3);
            }

//            tilt_position = ((gamepad1.left_stick_y + 1)/2);
//            tilt.setPosition(tilt_position);

//            close
            if (gamepad1.x) {
                grabber.setPosition(0);
            }
//          open
            if (gamepad1.b) {
                grabber.setPosition(0.7);
            }

            if (gamepad1.dpad_up) {
                arm.setPosition(0);
            }

            if (gamepad1.dpad_down) {
                arm.setPosition(0.9);
            }

//            if (gamepad1.dpad_up) {
//                arm_position = arm.getPosition();
//                arm.setPosition(arm_position + 0.005);
//            }
//
//            if (gamepad1.dpad_down) {
//                arm_position = arm.getPosition();
//                arm.setPosition(arm_position - 0.005);
//            }

//            if (gamepad1.right_bumper) {
//                arm.setPosition(arm_position);
//                tilt.setPosition(tilt_position);
//            }

            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}
