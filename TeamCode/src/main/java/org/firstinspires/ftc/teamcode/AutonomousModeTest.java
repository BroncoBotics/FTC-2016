package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorHTColor;
import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 9/27/16.
 */


@Autonomous(name="Autonomous Mode Test", group="Auto")
public class AutonomousModeTest extends AutonomousOpMode {

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        encoderInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDriveStraight(DRIVE_SPEED, 48, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
       // encoderTurn(1, 90, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoder(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
