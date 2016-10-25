/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.codelib;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoder(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class AutonomousOpMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime     runtime = new ElapsedTime();

    static double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static double     WHEEL_DIAMETER_INCHES   = 3.975 ;     // For figuring circumference
    static double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);


    DcMotor leftMotor, rightMotor;

    OpticalDistanceSensor lightSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    ColorSensor colorSensor;


    @Override
    public void runOpMode() {}

    public void hardwareInit() {
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "rangesensor");
        lightSensor = hardwareMap.opticalDistanceSensor.get("opticaldistance");

        colorSensor = hardwareMap.colorSensor.get("colorsensor");
        colorSensor.enableLed(false);

        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                leftMotor.getCurrentPosition(),
                rightMotor.getCurrentPosition());
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoder(double leftSpeed, double rightSpeed,
                        double leftInches, double rightInches,
                        double timeoutS) {
        int leftTarget;
        int rightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Determine new target position, and pass to motor controller
            leftTarget = (int)(Math.abs(leftInches) * COUNTS_PER_INCH);
            rightTarget = (int)(Math.abs(rightInches) * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();

            leftMotor.setPower(leftSpeed);
            rightMotor.setPower(rightSpeed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (Math.abs(leftMotor.getCurrentPosition()) < leftTarget  || Math.abs(rightMotor.getCurrentPosition()) < rightTarget)) {

                if (Math.abs(leftMotor.getCurrentPosition()) >= leftTarget) {
                    leftMotor.setPower(0);
                } else {
                    leftMotor.setPower(leftSpeed);
                }
                if (Math.abs(rightMotor.getCurrentPosition()) >= rightTarget) {
                    rightMotor.setPower(0);
                } else {
                    rightMotor.setPower(rightSpeed);
                }

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", leftTarget,  rightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            leftMotor.getCurrentPosition(),
                                            rightMotor.getCurrentPosition());
                telemetry.update();

                // Allow time for other processes to run.
                idle();
            }

            // Stop all motion;
            leftMotor.setPower(0);
            rightMotor.setPower(0);


            sleep(250);   // optional pause after each move
        }
    }

    public void encoderDriveStraight(double speed, double distance, double timeoutS) {
        encoder(speed, speed, distance, distance, timeoutS);
    }

    public void encoderTurn(double speed, double degrees, double timeoutS) {
        double ROTATION_k = 0.1875; //Inches per degree
        double leftSpeed = (degrees > 0) ? speed : -speed;
        double rightSpeed = (degrees > 0) ? -speed : speed;

        double distance = degrees * ROTATION_k;

        encoder(leftSpeed, rightSpeed, distance, distance, timeoutS);
    }

    public void driveToLine(double speed, double timeoutS) {

        double WHITE_THRESHOLD         = 0.25;

        runtime.reset();

        while ((lightSensor.getLightDetected() < WHITE_THRESHOLD) && runtime.seconds() < timeoutS) {

            // Start the robot moving forward, and then begin looking for a white line.
            leftMotor.setPower(speed);
            rightMotor.setPower(speed);

            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();

            idle();
        }

        leftMotor.setPower(0);
        rightMotor.setPower(0);


        lightSensor.enableLed(false);
    }

    public enum Turn {
        RIGHT_FAVORING, LEFT_FAVORING
    }

    public void followLine(double base_speed, double distanceInch, Turn turn_favorite, double timeoutS) {

        double     HEADING_CORRECTION_SPEED         = 0.4;
        double     WHITE_THRESHOLD                  = 0.25;

        // turn on LED of light sensor.
        lightSensor.enableLed(true);

        runtime.reset();


        while (rangeSensor.getDistance(DistanceUnit.INCH) > distanceInch && runtime.seconds() < timeoutS) {

            double light_intensity = lightSensor.getLightDetected();
            telemetry.addData("Light Level", light_intensity);
            telemetry.update();

            if (turn_favorite == Turn.LEFT_FAVORING) {
                HEADING_CORRECTION_SPEED = -HEADING_CORRECTION_SPEED;
            }


            if (light_intensity > WHITE_THRESHOLD) {
                //Sees white line
                leftMotor.setPower(base_speed);
                rightMotor.setPower(base_speed);
                encoderDriveStraight(.3, 2, 3);
            } else  {
                //Sees black tile
                leftMotor.setPower(HEADING_CORRECTION_SPEED);
                rightMotor.setPower(-HEADING_CORRECTION_SPEED);
            }

            telemetry.addData("RightMotor", rightMotor.getPower());
            telemetry.addData("LeftMotor", leftMotor.getPower());
            telemetry.update();

            idle();
        }


        leftMotor.setPower(0);
        rightMotor.setPower(0);

    }



    public enum Color {
        RED, BLUE
    }

    public Color identifyColor() {

        if (colorSensor.blue() > colorSensor.red())
            return Color.BLUE;
        else
            return Color.RED;
    }
}
