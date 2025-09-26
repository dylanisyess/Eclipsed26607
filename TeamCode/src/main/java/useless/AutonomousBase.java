package useless;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutonomousBase extends OpModeBase {
    protected RobotDriver robotDriver;
    final protected void setupAndWait() {
        robot.init(hardwareMap);
        robotDriver = new RobotDriver(robot, this);

        telemetry.addData("Heading", "%.4f", robot::getHeading);
        if (Math.abs(robot.getHeading()) > 20) {
            robot.beep(1);
        }

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.update();
            idle();
        }
    }

    @Config

    @Autonomous(name="pid auto", group="Robot")
    //@Disabled`
    public static class PID_auto extends LinearOpMode {
        private PIDController controller;

        public static double Kp = 0, Ki = 0, Kd = 0;
        public static double f = 0;
        public static int target = 0;
        private final double ticks_in_degree = 2000 / 270.0;

        private DcMotorEx leftwheel   = null;
        private DcMotorEx       rightwheel  = null;
        private Servo leftservo   = null;
        private Servo           rightservo  = null;
        private Servo           grabber     = null;
        private Servo           tilt        = null;
        private Servo           arm         = null;
        double integralSum;
        private double lasterror = 0;
        ElapsedTime timer = new ElapsedTime();

    //    @Override
        public void runOpMode() {
            controller = new PIDController(Kp, Ki, Kd);
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            leftwheel = hardwareMap.get(DcMotorEx.class, "leftwheel");
            rightwheel = hardwareMap.get(DcMotorEx.class, "rightwheel");
            leftservo = hardwareMap.get(Servo.class, "leftservo");
            rightservo = hardwareMap.get(Servo.class, "rightservo");
            grabber = hardwareMap.get(Servo.class, "grabber");
            tilt = hardwareMap.get(Servo.class, "tilt");
            arm = hardwareMap.get(Servo.class, "arm");
            leftwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftwheel.setDirection(DcMotor.Direction.REVERSE);
            rightwheel.setDirection(DcMotor.Direction.FORWARD);

            waitForStart();

            leftservo.setPosition(0.5);
            rightservo.setPosition(0.5);

            double power = PIDcontrol(357, leftwheel.getCurrentPosition());
            leftwheel.setPower(power);
            rightwheel.setPower(power);

        }

    //    @Override
        public double PIDcontrol(double reference, double state) {
            double error = reference - state;
            integralSum += error * timer.seconds();
            double derivative = (error - lasterror) / timer.seconds();
            lasterror = error;

            timer.reset();

            double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
            return output;
        }
    }

    @TeleOp(name="tank_drive", group="Linear OpMode")
    // @Disabled
    public static class tank_drive extends LinearOpMode {

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

    public static class Swerve_drive {
        public Swerve_drive(HardwareMap hardwareMap, double LX, double LY) {}

    }
}