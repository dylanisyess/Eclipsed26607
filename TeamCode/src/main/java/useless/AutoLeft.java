package useless;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto", preselectTeleOp = "Gamepad")
@Disabled
public class AutoLeft extends AutonomousBase {
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
        robotDriver.gyroDrive(0.6d, -3d, 0d, 5d, null);
        sleep(100);
        rightservo.setPosition(0.5);
        leftservo.setPosition(0.5);
        sleep(100);
        robotDriver.gyroDrive(0.6d, -10d, 0d, 5d, null);
    }

    @TeleOp(name="Arm", group="Linear OpMode")
    // @Disabled
    public static class Arm extends LinearOpMode {

        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        public Servo grabber;
        public Servo tilt;
        public Servo arm;
        private boolean grabbing;
        public Servo slide;
        public DcMotor intake;
        public double slide_position;
        public boolean taking;
        //    @Override
        public void runOpMode() {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            // Initialize the hardware variables.
            grabber  = hardwareMap.get(Servo.class, "grabber");
            tilt = hardwareMap.get(Servo.class, "tilt");
            arm = hardwareMap.get(Servo.class, "arm");
    //        slide = hardwareMap.get(Servo.class, "slide");
            intake = hardwareMap.get(DcMotor.class, "intake");

            // Set servo directions (keep default).
            grabber.setDirection(Servo.Direction.FORWARD);
            tilt.setDirection(Servo.Direction.FORWARD);
            arm.setDirection(Servo.Direction.FORWARD);
            taking = false;


            // Wait for the game to start (driver presses START).
            waitForStart();
            runtime.reset();

            grabbing = false;

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                if (gamepad1.x) {
                    tilt.setPosition(0.4);
                    sleep(100);
                    arm.setPosition(0.5);
                    sleep(100);
                    tilt.setPosition(0.7);
                    arm.setPosition(0);
                }

                if (gamepad1.y) {
                    arm.setPosition(0.1);
                    tilt.setPosition(0);
                }


                if (gamepad1.b) {
                    if (!grabbing) {
                        grabber.setPosition(0.4);
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



                // Telemetry to display key data
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("tilt Servo Position", tilt.getPosition());
                telemetry.addData("grabber Servo Position", grabber.getPosition());
                telemetry.addData("arm Servo Position", arm.getPosition());
    //            telemetry.addData("Linear Slide Position", slide.getPosition());
    //            telemetry.addData("intaking?", taking);
                telemetry.update();
            }
        }
    }
}