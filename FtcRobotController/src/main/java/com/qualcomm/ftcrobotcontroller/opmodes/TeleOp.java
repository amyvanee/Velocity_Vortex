package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends LinearOpMode {
    DcMotor wfl, wbl, wfr, wbr, arm;
    DcMotorController arm_controller;
    Servo thrower, wingL, wingR;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            arm = hardwareMap.dcMotor.get("arm");
            arm_controller = hardwareMap.dcMotorController.get("tetrix");
            arm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

            wfr = hardwareMap.dcMotor.get("wfr");
            wbr = hardwareMap.dcMotor.get("wbr");
            wfl = hardwareMap.dcMotor.get("wfl");
            wbl = hardwareMap.dcMotor.get("wbl");

            thrower = hardwareMap.servo.get("thrower");
            wingL = hardwareMap.servo.get("left_wing");
            wingR = hardwareMap.servo.get("right_wing");

            wbr.setDirection(DcMotor.Direction.REVERSE);
            wfr.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "hardware initialization error");
        }

        waitOneFullHardwareCycle();
        telemetry.addData("Ready", "Press Start to Begin!");
        waitForStart();

        while (opModeIsActive()) {
            float throttle = -gamepad1.right_stick_y;
            float direction = gamepad1.right_stick_x;
            float right = Range.clip(throttle - direction, -1, 1);
            float left = Range.clip(throttle + direction, -1, 1);

            setWheelPower(left, right);

            if (gamepad1.a)
                thrower.setPosition(1);
            else
                thrower.setPosition(0);

            arm.setPower(-gamepad1.left_stick_y);

            wingL.setPosition(Range.clip(gamepad1.left_trigger, -1, 1));
            wingR.setPosition(Range.clip(gamepad1.right_trigger, -1, 1));

            waitOneFullHardwareCycle();
        }
    }

    void setWheelPower(float left, float right) {
        // write the values to the motors
        wfr.setPower(right);
        wbr.setPower(right);
        wfl.setPower(left);
        wbl.setPower(left);
    }
}