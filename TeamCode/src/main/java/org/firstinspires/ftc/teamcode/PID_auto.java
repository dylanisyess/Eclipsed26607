package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// info
// 357 encoder counts for one tile (2 feet or 24 inches)

@Config

@Autonomous(name="Robot left", group="Robot")
//@Disabled
public class PID_auto extends LinearOpMode {
    private PIDController controller;

    public static double Kp = 0, Ki = 0, Kd = 0;
    public static double f = 0;
    public static int target = 0;
    private final double ticks_in_degree = 2000 / 270.0;

    private DcMotorEx       leftwheel   = null;
    private DcMotorEx       rightwheel  = null;
    private Servo           leftservo   = null;
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
        leftwheel  = hardwareMap.get(DcMotorEx.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotorEx.class, "rightwheel");
        leftservo = hardwareMap.get(Servo.class, "leftservo");
        rightservo = hardwareMap.get(Servo.class, "rightservo");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");
        leftwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightwheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();


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
