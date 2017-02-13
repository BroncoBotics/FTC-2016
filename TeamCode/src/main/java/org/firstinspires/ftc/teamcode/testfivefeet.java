package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by davidkrakauer on 1/21/17.
 */
@Autonomous(name="Test 360 Turns", group="Auto")
@Disabled
public class testfivefeet extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.35;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderTurn(TURN_SPEED,360,10);
        sleep(2000);
        encoderTurn(-TURN_SPEED,-360,10);
    }



}
