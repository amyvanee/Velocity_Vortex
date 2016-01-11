package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.Telemetry;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class AutoRed extends BotHardware {
    private short state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("State", state);
            telemetry.addData("Z", gyro.getIntegratedZValue());
            telemetry.addData("Sonic Dist", sonar.getUltrasonicLevel());

            switch (state) {
                case 0://global start drive
                case 4:
                    driveGyroStart(0.25f);
                    state++;
                    break;


                case 1://drive 2 seconds
                    driveGyro(0.25f);
                    if(getTime() >= 2000){
                        setPower(0);
                        state++;
                    }
                    break;

                case 5://drive until white line
                    driveGyro(0.5f);
                    if(isOnLine(false)){
                        setPower(0);
                        state++;
                    }
                    break;

                case 2://global left turn start
                case 6:
                    turnGyroStart(-0.25f, 0.25f);
                    state++;
                    break;

                case 3://global turn 45
                case 7:
                    if(turnGyro(45)){
                        setPower(0);
                        state++;
                    }
                    break;

                //now facing wall

                case 8://fixes throw distance and dumps climbers
                    if(correctThrowDist()){ //also moves robot
                        dumpClimbers();
                        state++;
                    }
                    break;

                case 9:
                    if(correctBeaconDist()){
                        setUpBeaconPress(); //reads on sets servo
                        Thread.sleep(1500);//to allow servo to position its self
                        state++;
                        setPower(0.15f);
                    }
                    break;

                case 10:
                    if(hasPressedBeacon()){
                        setPower(0);
                        state++;
                    }
                    break;

                default:
                    done();
            }
        }
    }
}
