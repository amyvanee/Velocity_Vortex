package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;


public class TeleOp extends LinearOpMode {
    DcMotor wfl, wbl, wfr, wbr, arm;
    @Override
    public void runOpMode() throws InterruptedException {
        try {
            wfr = hardwareMap.dcMotor.get("wfr");
            wbr = hardwareMap.dcMotor.get("wbr");
            wfl = hardwareMap.dcMotor.get("wfl");
            wbl = hardwareMap.dcMotor.get("wbl");

            wbr.setDirection(DcMotor.Direction.REVERSE);
            wfr.setDirection(DcMotor.Direction.REVERSE);

            wfr.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wbr.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wfl.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            wbl.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "driving wheels setup");
        }
        try {
            arm = hardwareMap.dcMotor.get("arm");
            arm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "arm motor and controller setup");
        }

        setPower(0f);

        // power values
        float wheelPower, turnPower;
        float armPower;

        waitForStart();

        while (opModeIsActive()) {
            // ----------------
            // gamepad1:
            // ----------------

            wheelPower = gamepad1.left_stick_y;
            turnPower = -gamepad1.right_stick_x;

            if (gamepad1.right_bumper) {
                wheelPower = scaleInput(wheelPower) / 2;
                turnPower = scaleInput(turnPower) / 2;
            } else if(gamepad1.left_bumper) {
                wheelPower = scaleInput(wheelPower) / 4;
                turnPower = scaleInput(turnPower) / 4;
            } else {
                wheelPower = scaleInput(wheelPower);
                turnPower = scaleInput(turnPower);
            }

            // allows for tighter point turns
            if (Math.abs(turnPower) > 0)
                setPower(-turnPower, turnPower);
            else
                setPower(wheelPower);

            // --------------
            // gamepad2;
            // --------------

            armPower = gamepad2.right_stick_y;
            // must hold down 'a' before being able to move arm (a for activate)
            if (gamepad2.a) {
                armPower = -gamepad2.left_stick_y;
                armPower = scaleInput(armPower);
            } else {
                armPower = 0f;
            }

            if (gamepad2.right_bumper){
                armPower = scaleInput(armPower)/2;
            }

            arm.setPower(armPower);

//            if(gamepad1.a)
//                thrower.setPosition(1);
//            else
//                thrower.setPosition(0);

//            // controlling the wings
//            leftWing.setPosition(Range.scale(gamepad2.left_trigger, 0, 1, 1, 0.2));
//            rightWing.setPosition(Range.scale(gamepad2.right_trigger, 0, 1, 0.2, 1));

            waitOneFullHardwareCycle();
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
}
