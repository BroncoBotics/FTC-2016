package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 10/7/16.
 */
@Autonomous(name="First Autonomous Mode", group="Auto")
public class MyFirstAuto extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.6;
    static double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDriveStraight(DRIVE_SPEED, 15, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderTurn(TURN_SPEED, 45, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDriveStraight(DRIVE_SPEED, 40, 5.0);
        driveToLine(0.05, 2);
        followLine(0.05, 3, Turn.RIGHT_FAVORING, .5, 10);
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

        sleep(4000);
    }
}
