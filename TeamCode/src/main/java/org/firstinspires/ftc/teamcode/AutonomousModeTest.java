package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 9/27/16.
 */

@Disabled
@Autonomous(name="Autonomous Mode Test", group="Auto")
public class AutonomousModeTest extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.6;
    static double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {

        hardwareInit();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

//        encoderDriveStraight(.5 ,10, 20.0);
        encoderTurn(1.0,90, 20);

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
}
