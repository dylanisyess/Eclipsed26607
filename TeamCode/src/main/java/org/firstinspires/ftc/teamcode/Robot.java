package org.firstinspires.ftc.teamcode;;

import android.content.Context;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.Arrays;
import java.util.List;

public class Robot {
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public Servo frontLeftServo, frontRightServo, backLeftServo, backRightservo;
    public DigitalChannel limitSwitch;
    private List<DcMotorEx> motors;
    private Context _appContext;
    private ElapsedTime runtime = new ElapsedTime();
    public double rightpodposition;
    public double leftpodposition;
    public boolean leftpoddirection;
    public boolean rightpoddirection;
    public double left_theta, right_theta, prev_left_theta, prev_right_theta;
    public double left_magnitude, right_magnitude;
    public double left_forward, right_forward;
    public boolean moving;
    public double magnitude_gain = 0.5;
    public float rightwheelposition;

    IMU imu;
    private static final double MAX_VELOCITY = 2800d;
    private static final double COUNTS_PER_MOTOR_REV = 146.44d;    // eg: HD Hex Motor 20:1 560, core hex 288, 40:1 1120
    private static final double DRIVE_GEAR_REDUCTION = 1.33d;     // This is < 1.0 if geared UP, eg. 26d/10d
    private static final double WHEEL_DIAMETER_INCHES = 2.3622d;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359d);
    double gripPosition, handPosition;
    int handMode;

    public void init(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        motors = Arrays.asList(frontLeft, frontRight, backLeft, backRight);

        frontLeftServo = hardwareMap.get(Servo.class, "frontLeftServo");
        frontRightServo = hardwareMap.get(Servo.class, "frontRightServo");
        backLeftServo = hardwareMap.get(Servo.class, "backLeftServo");
        backRightServo = hardwareMap.get(Servo.class, "backRightServo");

        frontLeft.setDirection(Direction.REVERSE);
        frontRight.setDirection(Direction.FORWARD);
        backLeft.setDirection(Direction.REVERSE);
        backRight.setDirection(Direction.FORWARD);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LogoFacingDirection.RIGHT, UsbFacingDirection.FORWARD)));
        imu.resetYaw();

        _appContext = hardwareMap.appContext;
       
       for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.setDriveZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }

    }


    double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    void resetDrive() {
        setDriveMode(RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(RunMode.RUN_WITHOUT_ENCODER);
        setDriveZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }


    void setDriveTarget(double distance, boolean moveSideway) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        int dirFL = moveSideway ? -1 : 1;
        int dirFR = 1;
        int dirRL = 1;
        int dirRR = moveSideway ? -1 : 1;

        int leftFrontTarget = leftwheel.getCurrentPosition() + moveCounts * dirFL;
        int rightFrontTarget = rightwheel.getCurrentPosition() + moveCounts * dirFR;

        // Set Target and Turn On RUN_TO_POSITION
        leftwheel.setTargetPosition(leftFrontTarget);
        rightwheel.setTargetPosition(rightFrontTarget);
    }

    boolean isDriveBusy() {
        return leftwheel.isBusy() && rightwheel.isBusy();
    }
}