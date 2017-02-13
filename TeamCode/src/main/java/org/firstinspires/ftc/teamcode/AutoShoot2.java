package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 10/7/16.
 */
@Autonomous(name="Corner Position Shoot", group="Auto")
public class AutoShoot2 extends AutonomousOpMode{

    static double     DRIVE_SPEED             = 0.35;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        // S1: Forward 47 Inches with 5 Sec timeout
        sleep(3000);
        shootOn();
        sleep(7000);
        shootOff();
//        followLine(0.4, 4, Turn.RIGHT_FAVORING, 20);
//        encoderTurn(TURN_SPEED, 4, 5.0);
//        Color color = identifyColor();


//        String colorString;
//
//
//        if (color == Color.BLUE) {
//            colorString = "Blue";
//            encoderDriveStraight(.2, 2, 1.0);
//        } else {
//            colorString = "Red";
//            encoderTurn(.4, -4, 5.0);
//            encoderDriveStraight(.2, 3, 1.0);
//        }


//
////
//        telemetry.addData("Path", "Complete");
//        telemetry.addData("Color", colorString);
//        telemetry.update();

        sleep(3000);
    }



}
