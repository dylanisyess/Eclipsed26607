package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="tank_drive", group="Linear OpMode")
// @Disabled
public class tank_drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo grabber;
    public Servo tilt;
    public double tilt_position;
    public Servo arm;
    public double arm_position;
    private boolean grabbing;
    //    public Servo slide;
    public DcMotor intake;
    public double slide_position;
    public boolean taking;
    public Servo leftservo;
    public Servo rightservo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");


        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
            double leftPower;
            double rightPower;

            leftservo.setPosition(0.5);
            rightservo.setPosition(0.5);

            // Map the joystick inputs to motor power
            leftPower  = -gamepad1.left_stick_y * 0.8;
            rightPower = -gamepad1.right_stick_y * 0.8;

            // Send calculated power to wheels
            leftwheel.setPower(leftPower);
            rightwheel.setPower(rightPower);

            if (gamepad1.x) {
                tilt.setPosition(0.2);
                sleep(100);
                arm.setPosition(0.5);
                sleep(100);
                arm.setPosition(1);
                sleep(700);
                tilt.setPosition(0.7);
                arm.setPosition(1);
            }

            if (gamepad1.y) {
                arm.setPosition(0);
                tilt.setPosition(0);
            }


            if (gamepad1.b) {
                if (!grabbing) {
                    grabber.setPosition(0.9);
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

            if (gamepad1.dpad_down) {
                intake.setPower(-1);
            }

            if (gamepad1.dpad_up) {
                leftservo.setPosition(0.5);
                rightservo.setPosition(0.5);
            }


            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}