package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;


public class TeleOp extends BotHardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

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
            } else if (gamepad1.left_bumper) {
                wheelPower = scaleInput(wheelPower) / 4;
                turnPower = scaleInput(turnPower) / 4;
            } else {
                wheelPower = scaleInput(wheelPower);
                turnPower = scaleInput(turnPower);
            }

            // allows for tighter point turns
            if (Math.abs(turnPower) > 0.05f)
                setPower(turnPower, -turnPower);
            else
                setPower(-wheelPower);

            // --------------
            // gamepad2;
            // --------------

            armPower = gamepad2.right_stick_y;
            // must hold down 'a' before being able to move arm (a for activate)
            if (gamepad2.a) {
                armPower = gamepad2.left_stick_y;
                armPower = scaleInput(armPower);
            } else {
                armPower = 0f;
            }

            if (gamepad2.right_bumper) {
                armPower = scaleInput(armPower) / 2;
            }

            arm.setPower(armPower);

            if(gamepad1.a)
                thrower.setPosition(1);
            else
                thrower.setPosition(0);

            // controlling the wings
            leftWing.setPosition(Range.scale(gamepad2.left_trigger, 0, 1, 1, 0.2));
            rightWing.setPosition(Range.scale(gamepad2.right_trigger, 0, 1, 0.2, 1));

            waitOneFullHardwareCycle();
        }
    }
}
