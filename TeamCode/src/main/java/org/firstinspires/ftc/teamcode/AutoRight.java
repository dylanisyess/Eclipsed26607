package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "Auto Right", preselectTeleOp = "Gamepad")
//@Disabled
public class AutoRight extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        setupAndWait();
        rightservo.setPosition(1);
        leftservo.setPosition(1);
        sleep(100);
        robotDriver.gyroDrive(0.6d, 3d, 0d, 5d, null);
        sleep(100);
        rightservo.setPosition(0.5)
        leftservo.setPosition(0.5)
        sleep(100);
        robotDriver.gyroDrive(0.6d, -10d, 0d, 5d, null);
    }
}