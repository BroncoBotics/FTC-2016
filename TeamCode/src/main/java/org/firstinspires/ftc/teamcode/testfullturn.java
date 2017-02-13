package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by DKrakauer on 21/1/17.
 */
@Autonomous(name="TestTurn", group="Auto")
@Disabled
public class testfullturn extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.35;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderTurn(TURN_SPEED,360,10);

    }



}
