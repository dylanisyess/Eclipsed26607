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
    public Servo arm;
    private boolean grabbing;

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

        grabbing = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x) {
                arm.setPosition(0.7);
                tilt.setPosition(0);
            }

            if (gamepad1.y) {
                arm.setPosition(0);
                tilt.setPosition(0.5);
            }


            if (gamepad1.a) {
                arm.setPosition(1);
                tilt.setPosition(0);
            }

            if (gamepad1.b) {
                if (!grabbing) {
                    grabber.setPosition(0);
                    grabbing = true;
                } else {
                    grabber.setPosition(1);
                    grabbing = false;
                }
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
