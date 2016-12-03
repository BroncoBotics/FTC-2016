package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.codelib.AutonomousOpMode;

/**
 * Created by JackV on 10/7/16.
 */
@Autonomous(name="Autonomous Mode #1", group="Auto")
public class MyFirstAuto extends AutonomousOpMode {

    static double     DRIVE_SPEED             = 0.35;
    static double     TURN_SPEED              = 1.0;

    @Override
    public void runOpMode() {

        hardwareInit();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDriveStraight(DRIVE_SPEED, 15, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        setTelemetry("Driving: ", "Straight");
        encoderTurn(TURN_SPEED, 55, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        setTelemetry("Driving: ", "Right");
        encoderDriveStraight(DRIVE_SPEED, 30, 5.0);
        setTelemetry("Driving: ", "Straight");
        driveToLine(0.3, 5);
        setTelemetry("Driving: ", "to Line");
        followLine(0.4, 4, Turn.RIGHT_FAVORING, 20);
        setTelemetry("Driving: ", "Following Line");
        encoderTurn(TURN_SPEED, 4, 5.0);
        setTelemetry("Driving: ", "Approximating Line...");
        Color color = identifyColor();


        String colorString;


        if (color == Color.BLUE) {
            colorString = "Blue";
//            encoderDriveStraight(.2, 2, 1.0);
        } else {
            colorString = "Red";
//            encoderTurn(.4, -4, 5.0);
//            encoderDriveStraight(.2, 3, 1.0);
        }

        setTelemetry("Color Result", colorString);

//
////
//        telemetry.addData("Path", "Complete");
//        telemetry.addData("Color", colorString);
//        telemetry.update();

        sleep(4000);
    }



    private void setTelemetry(String message1, String message2){
        telemetry.addData(message1, message2);
        telemetry.update();
    }



}
