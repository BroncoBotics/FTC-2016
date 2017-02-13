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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.codelib.Drivetrain;

@TeleOp(name="Two Drivers", group="Teleop")  // @Autonomous(...) is the other common choice
public class TeleopTwoController extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // private DcMotor leftMotor = null;
    // private DcMotor rightMotor = null;

//   AnalogInput limitSwitchOut, limitSwitchIn;

    Drivetrain drive;
    private DcMotor lifts, suckerRight, suckerLeft, lShoot, rShoot, slides;
    boolean slowMode = false, hasDriveCheckRunOnce = false,
            suckerOn = false, hasSuckerCheckRunOnce = false,
            flywheelsOn = false, haveFlyWheels = false;

    boolean liftsOn = false,setLifts = false;

    Servo buttonPush;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap.dcMotor.get("left motor"), hardwareMap.dcMotor.get("right motor"));

//        limitSwitchOut = hardwareMap.analogInput.get("limitSwitchout");
//        limitSwitchIn = hardwareMap.analogInput.get("limitSwitchin");

        //motors
//        lifts = hardwareMap.dcMotor.get("slides");
//        suckerRight = hardwareMap.dcMotor.get("collectorRight");
//        suckerRight.setDirection(DcMotorSimple.Direction.REVERSE);
//        suckerLeft = hardwareMap.dcMotor.get("collectorLeft");
        slides = hardwareMap.dcMotor.get("lifts");

        lShoot = hardwareMap.dcMotor.get("lshoot");
//        rShoot = hardwareMap.dcMotor.get("rshoot");
        rShoot.setDirection(DcMotorSimple.Direction.REVERSE);

        buttonPush = hardwareMap.servo.get("lservo");



        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {

    }

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
        if (gamepad1.a && !hasDriveCheckRunOnce) {
            slowMode = !slowMode;
            hasDriveCheckRunOnce = true;
        }
        if (!gamepad1.a)
            hasDriveCheckRunOnce = false;

        if (slowMode) {
            drive.arcadeDrive(-gamepad1.left_stick_y * (float).25, gamepad1.right_stick_x * (float).6, false);
        } else {
            drive.arcadeDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, true);
        }

        // Sucker Control
        if (gamepad2.x && !hasSuckerCheckRunOnce) {
            suckerOn = !suckerOn;
            hasSuckerCheckRunOnce = true;
        }
        if (!gamepad2.x)
            hasSuckerCheckRunOnce = false;



        if(gamepad2.a && !liftsOn) {
            setLifts = !setLifts;
            liftsOn = true;
        }
        if(!gamepad2.a){
            liftsOn = false;
        }

        suckerRight.setPower((suckerOn) ? 1.0 : 0.0);
        suckerLeft.setPower((suckerOn) ? 1.0 : 0.0);

        lifts.setPower((setLifts) ? 1.0 : 0.0);

        // Flywheels Control
        if (gamepad2.b && !haveFlyWheels) {
            flywheelsOn = !flywheelsOn;
            haveFlyWheels = true;
        }
        if (!gamepad2.b)
            haveFlyWheels = false;

        lShoot.setPower((flywheelsOn) ? 0.95 : 0.0);
        rShoot.setPower((flywheelsOn) ? 0.95 : 0.0);

//        if (!limitSwitched("Out") && !limitSwitched("In")) {
//            lifts.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
//        } else {
//            lifts.setPower(0);
//        }
//        }


        if(gamepad2.right_bumper){
            buttonPush.setPosition(0.4);
        } else {
            buttonPush.setPosition(0);
        }

        if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1){
            slides.setPower(gamepad2.left_stick_y);
        }else {
            slides.setPower(0);
        }

    }

    @Override
    public void stop() {
        drive.stop();
        suckerLeft.setPower(0);
        suckerRight.setPower(0);
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
