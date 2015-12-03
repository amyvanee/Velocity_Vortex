package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class TeleOp extends LinearOpMode
{
    DcMotor wfl, wbl, wfr, wbr, arm;
    DcMotorController arm_controller;
    Servo thrower;

    @Override
    public void runOpMode() throws InterruptedException
    {
        try
        {
            initHardware();
        }
        catch (Exception e)
        {
            telemetry.addData("[ERROR]:", "hardware initialization error");
        }

        float wheelPower, turnPower; // power values
        float armPower;

        waitForStart();

        while (opModeIsActive())
        {
            // ----------------
            // gamepad1:
            // ----------------

            // using seperate control scheme because it might help with climbing
            wheelPower = -gamepad1.left_stick_y;
            turnPower = -gamepad1.right_stick_x;

            wheelPower = scaleInput(Range.clip(wheelPower, -1, 1));
            turnPower = scaleInput(Range.clip(turnPower, -1, 1));

            // allows for tighter point turns
            if (Math.abs(turnPower) > 0) {
                setWheelPower(-turnPower, turnPower);
            }
            else if (gamepad1.b)
            {
                setWheelPower(wheelPower, wheelPower);
                arm.setPower(-wheelPower);
            }
            else
            {
                setWheelPower(wheelPower, wheelPower);
            }

            if(gamepad1.a)
                thrower.setPosition(1);
            else
                thrower.setPosition(0);

            // --------------
            // gamepad2;
            // --------------

            // must hold down a before being able to move arm
            if (gamepad2.a)
            {
                armPower = -gamepad2.left_stick_y;
                armPower = scaleInput(Range.clip(armPower, -1, 1));
                if (armPower > 0)
                    telemetry.addData("Arm", "Extending");
                else
                    telemetry.addData("Arm", "Retracting");
            }
            else
                armPower = 0f;

            arm.setPower(armPower);

            // controlling the wings

            /*
            if (gamepad2.left_bumper)
            {
                leftWing.setPosition(1);
                telemetry.addData("Left Wing", "Extended");
            }
            else
            {
                leftWing.setPosition(0);
                telemetry.addData("Left Wing", "Retracted");
            }

            if (gamepad2.right_bumper)
            {
                rightWing.setPosition(1);
                telemetry.addData("Right Wing", "Extended");
            }
            else
            {
                rightWing.setPosition(0);
                telemetry.addData("Right Wing", "Retracted");
            }
            */

            waitOneFullHardwareCycle();
        }
    }

    void initHardware()
    {
        arm = hardwareMap.dcMotor.get("arm");
        arm_controller = hardwareMap.dcMotorController.get("tetrix");
        arm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        wfr = hardwareMap.dcMotor.get("wfr");
        wbr = hardwareMap.dcMotor.get("wbr");
        wfl = hardwareMap.dcMotor.get("wfl");
        wbl = hardwareMap.dcMotor.get("wbl");

        thrower = hardwareMap.servo.get("thrower");
        leftWing = hardwareMap.servo.get("left_wing");
        rightWing = hardwareMap.servo.get("right_wing");

        wbr.setDirection(DcMotor.Direction.REVERSE);
        wfr.setDirection(DcMotor.Direction.REVERSE);
    }

    void setWheelPower(float left, float right)
    {
        // write the values to the motors
        wfr.setPower(right);
        wbr.setPower(right);
        wfl.setPower(left);
        wbl.setPower(left);
    }

    // experimental scaling method for controller:
    float scaleInput(float input)
    {
        if (input > 0)
            return (float)Math.pow(input, 2);
        return -(float)Math.pow(input, 2);
    }
}
