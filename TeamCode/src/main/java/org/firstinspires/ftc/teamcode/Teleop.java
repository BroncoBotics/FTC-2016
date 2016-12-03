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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.codelib.Drivetrain;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Teleop Main", group="Teleop")  // @Autonomous(...) is the other common choice
public class Teleop extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

//   AnalogInput limitSwitchOut, limitSwitchIn;

    Drivetrain drive;
    private DcMotor lifts, sucker, lShoot, rShoot;
    boolean slowMode = false, hasDriveCheckRunOnce = false,
            suckerOn = false, hasSuckerCheckRunOnce = false,
            stopperOn = false, hasStopperstopedthestop = false,
            flywheelsOn = false, haveFlyWheels = false;

    Servo stopper;

    //lifts - x up, a down

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap.dcMotor.get("left motor"), hardwareMap.dcMotor.get("right motor"));

//        limitSwitchOut = hardwareMap.analogInput.get("limitSwitchout");
//        limitSwitchIn = hardwareMap.analogInput.get("limitSwitchin");

        //motors
        lifts = hardwareMap.dcMotor.get("lifts");
        sucker = hardwareMap.dcMotor.get("sucker");
       // zipper = hardwareMap.dcMotor.get("zipper");

        lShoot = hardwareMap.dcMotor.get("lshoot");
        rShoot = hardwareMap.dcMotor.get("rshoot");
        rShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        stopper = hardwareMap.servo.get("stopper");


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        // Drivetrain Control
//        if (gamepad1.a && !hasDriveCheckRunOnce) {
//            slowMode = !slowMode;
//            hasDriveCheckRunOnce = true;
//        }
//        if (!gamepad1.a)
//            hasDriveCheckRunOnce = false;
//        if (slowMode) {
//            drive.arcadeDrive(-gamepad1.left_stick_y * (float)0.25, -gamepad1.right_stick_x, true);
//        } else {
//            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, true);
//        }


        drive.setPower(gamepad1.left_stick_y,gamepad1.right_stick_y);
        // Sucker Control
        if (gamepad1.x && !hasSuckerCheckRunOnce) {
            suckerOn = !suckerOn;
            hasSuckerCheckRunOnce = true;
        }
        if (!gamepad1.x)
            hasSuckerCheckRunOnce = false;

        sucker.setPower((suckerOn) ? 1.0 : 0.0);
      //  zipper.setPower((suckerOn) ? 1.0 : 0.0);

        // Flywheels Control
        if (gamepad1.b && !haveFlyWheels) {
            flywheelsOn = !flywheelsOn;
            haveFlyWheels = true;
        }
        if (!gamepad1.b)
            haveFlyWheels = false;

        lShoot.setPower((flywheelsOn) ? 0.95 : 0.0);
        rShoot.setPower((flywheelsOn) ? 0.95 : 0.0);

//        if (!limitSwitched("Out") && !limitSwitched("In")) {
            lifts.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//        } else {
//            lifts.setPower(0);
//        }
//        }


        //stopper code
        if (gamepad1.y && !hasStopperstopedthestop) {
            stopperOn = !stopperOn;
            hasStopperstopedthestop = true;
        }
        if (!gamepad1.y)
            hasStopperstopedthestop = false;

        stopper.setPosition((stopperOn) ? 0.5 : 0.0);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        drive.stop();
        sucker.setPower(0);
        lifts.setPower(0);
    }

//   private Boolean limitSwitched(String inout){
//       if(inout.equals("Out"))
//       return limitSwitchOut.getVoltage() > (.1 * limitSwitchOut.getMaxVoltage());
//
//       if(inout.equals("In"))
//       return limitSwitchIn.getVoltage() > (.1 * limitSwitchIn.getMaxVoltage());
//
//       return true;
//   }

}
