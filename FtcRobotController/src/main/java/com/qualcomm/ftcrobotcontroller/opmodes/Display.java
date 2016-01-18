package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class Display extends BotHardware {
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();
        setPower(-0.5f, 0.5f);
        while (opModeIsActive()) {
            if(getTime() > 10){
                setTime();
            }
            if((int)(getTime() % 2) == 0){
                leftWing.setPosition(1);
                rightWing.setPosition(1);
                thrower.setPosition(1);
                setPower(-0.5f, 0.5f);
            } else {
                leftWing.setPosition(0);
                rightWing.setPosition(0);
                thrower.setPosition(0);
                setPower(0.5f, -0.5f);
            }
        }
    }
}
