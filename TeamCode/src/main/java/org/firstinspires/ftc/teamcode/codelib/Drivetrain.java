package org.firstinspires.ftc.teamcode.codelib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Created by JackV on 9/26/16.
 */
public class Drivetrain {

    DcMotor left_motor, right_motor;

    public Drivetrain(DcMotor left_motor, DcMotor right_motor) {
        this.left_motor = left_motor;
        this.right_motor = right_motor;
        this.left_motor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        this.right_motor.setDirection(DcMotor.Direction.REVERSE);
        this.left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int[] getPosition(){

        int[] returnVal = {this.right_motor.getCurrentPosition(),this.left_motor.getCurrentPosition()};
        return returnVal;
    }

    public void setPower(double power) {
        left_motor.setPower(power);
        right_motor.setPower(power);
    }

    public void setPower(double left_power, double right_power) {
        left_motor.setPower(left_power);
        right_motor.setPower(right_power);
    }

    public void arcadeDrive(float moveValue, float rotateValue, boolean squaredInputs) {
        float leftMotorSpeed, rightMotorSpeed;

        moveValue = limit(moveValue);
        rotateValue = limit(rotateValue);

        if (squaredInputs) {
            // square the inputs (while preserving the sign) to increase fine control while permitting full power
            if (moveValue >= 0.0) {
                moveValue = (moveValue * moveValue);
            } else {
                moveValue = -(moveValue * moveValue);
            }
            if (rotateValue >= 0.0) {
                rotateValue = (rotateValue * rotateValue);
            } else {
                rotateValue = -(rotateValue * rotateValue);
            }
        }

        if (moveValue > 0.0) {
            if (rotateValue > 0.0) {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = Math.max(moveValue, rotateValue);
            } else {
                leftMotorSpeed = Math.max(moveValue, -rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            }
        } else {
            if (rotateValue > 0.0) {
                leftMotorSpeed = -Math.max(-moveValue, rotateValue);
                rightMotorSpeed = moveValue + rotateValue;
            } else {
                leftMotorSpeed = moveValue - rotateValue;
                rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
            }
        }

        left_motor.setPower(leftMotorSpeed);
        right_motor.setPower(rightMotorSpeed);
    }

    public void stop() {
        left_motor.setPower(0);
        right_motor.setPower(0);
    }

    protected static float limit(float num) {
        if (num > 1.0) {
            return 1;
        }
        if (num < -1.0) {
            return -1;
        }
        return num;
    }

//    public void moveToDistance(double distance, double speed) {
//
//        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        int adjusted_distance_right = right_motor.getCurrentPosition() + (int)(distance * 1120/(4 * Math.PI));
//        int adjusted_distance_left = left_motor.getCurrentPosition() + (int)(distance * 1120/(4 * Math.PI));
//
//        left_motor.setTargetPosition(adjusted_distance_left);
//        right_motor.setTargetPosition(adjusted_distance_right);
//
//        left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        left_motor.setPower(Math.abs(speed));
//        right_motor.setPower(Math.abs(speed));
//
//    }
//
//    public boolean areMotorsBusy() {
//        return left_motor.isBusy() || right_motor.isBusy();
//    }
//
//    public void turnForDistance(double distance) {
//        int adjusted_distance = (int)(distance * 360);
//        left_motor.setTargetPosition(adjusted_distance);
//        right_motor.setTargetPosition(-adjusted_distance);
//    }
//
//    public void turnForDistance(double distance, double speed) {
//        int adjusted_speed = (int)(speed * 360);
//        left_motor.setMaxSpeed(adjusted_speed);
//        right_motor.setMaxSpeed(adjusted_speed);
//        turnForDistance(distance);
//    }
}
