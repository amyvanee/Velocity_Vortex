package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * IMPORTANT: do not move robot after hitting init
 */
public class BotHardware extends LinearOpMode {
    final protected float autoPower = 0.5f;
    final protected double wheelCircumference = 5 * 3.1416;

    protected DcMotor wfl, wbl, wfr, wbr, arm;
    protected Servo thrower, leftWing, rightWing;
    protected ColorSensor groundLeft, groundRight, beacon;
    private GyroSensor gyro;

    void initHardware() {
        try {
            arm = hardwareMap.dcMotor.get("arm");
            arm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "arm motor and controller setup");
        }

        try {
            wfr = hardwareMap.dcMotor.get("wfr");
            wbr = hardwareMap.dcMotor.get("wbr");
            wfl = hardwareMap.dcMotor.get("wfl");
            wbl = hardwareMap.dcMotor.get("wbl");

            wbr.setDirection(DcMotor.Direction.REVERSE);
            wfr.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "driving wheels setup");
        }

        try {
            thrower = hardwareMap.servo.get("thrower");
            leftWing = hardwareMap.servo.get("left_wing");
            rightWing = hardwareMap.servo.get("right_wing");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "servo setup");
        }

        try {
            groundLeft = hardwareMap.colorSensor.get("ground_left");
            groundRight = hardwareMap.colorSensor.get("ground_right");
            beacon = hardwareMap.colorSensor.get("beacon");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "color sensor setup");
        }

        try {
            gyro = hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "gyro sensor setup");
        }
    }

    void setPower (float power) {
        wfr.setPower(power);
        wbr.setPower(power);
        wfl.setPower(power);
        wbl.setPower(power);
    }

    void setPower(float left, float right) {
        wfr.setPower(right);
        wbr.setPower(right);
        wfl.setPower(left);
        wbl.setPower(left);
    }

    float scaleInput(float input) {
        telemetry.addData("[TEST]:", "calling scale input");
        input = Range.clip(input, -1, 1);
        if (input > 0)
            return (float)Math.pow(input, 4);
        return -(float)Math.pow(input, 4);
    }

    // distance is in inches
    void driveDistance(float power, float distance) {
        resetEncoders();
        enableEncoders(true);
        setPower(power);
        int pos = (int)((distance / wheelCircumference) * 1120);
        while (Math.abs(wbl.getCurrentPosition()) < pos) {
            telemetry.addData("Motor Position", Math.abs(wbl.getCurrentPosition()));
            telemetry.addData("Motor Target", pos);
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                telemetry.addData("[ERROR]:", "distance wait");
            }
        }
        setPower(0f);
        enableEncoders(false);
    }

    void resetEncoders() {
        wfr.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        wbr.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        wfl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        wbl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
    }

    void enableEncoders(boolean encoder) {
        if (encoder) {
            wfr.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            wbr.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            wfl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
            wbl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        } else {
            wfr.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wbr.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wfl.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wbl.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // do nothing for now
    }
}
