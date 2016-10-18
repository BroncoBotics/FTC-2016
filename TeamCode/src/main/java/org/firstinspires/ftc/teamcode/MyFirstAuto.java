package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by JackV on 10/7/16.
 */
@Autonomous(name="First Autonomous Mode", group="Auto")
public class MyFirstAuto extends AutonomousModeTest {



    @Override
    public void runOpMode() {

        encoderInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDriveStraight(-DRIVE_SPEED, 15, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderTurn(TURN_SPEED, 90, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        driveToLine(0.5, 6);
        followLine(0.5, 2, Turn.RIGHT_FAVORING, 5);
        Color color = identifyColor();

        String colorString;


        if (color == Color.BLUE)
            colorString = "Blue";
        else
            colorString = "Red";



//
        telemetry.addData("Path", "Complete");
        telemetry.addData("Color", colorString);
        telemetry.update();
    }
}
