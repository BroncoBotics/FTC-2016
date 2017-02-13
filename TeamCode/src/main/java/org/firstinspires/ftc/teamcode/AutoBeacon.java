package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 10/7/16.
 */
@Autonomous(name="Just Beacon?", group="Auto")
public class AutoBeacon extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.35;
    static double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        encoderDriveStraight(DRIVE_SPEED, 5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        shootOn();
//        sleep(5000);
//        shootOff();
        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
       // encoderDriveStraight(DRIVE_SPEED, 25, 3.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDriveStraight(DRIVE_SPEED, 15,5);
        encoderTurn(TURN_SPEED, 35, 5.0);
        encoderDriveStraight(DRIVE_SPEED, 40, 5.0);
        encoderTurn(TURN_SPEED, 45, 5.0);
//        driveToLine(0.5, 5.0);







//        driveToLine(0.3, 5);
//        followLine(0.4, 4, Turn.RIGHT_FAVORING, 20);


//        encoderTurn(TURN_SPEED, 4, 5.0);
//        Color color = identifyColor();
//        String colorString;
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

        sleep(4000);
    }



}
