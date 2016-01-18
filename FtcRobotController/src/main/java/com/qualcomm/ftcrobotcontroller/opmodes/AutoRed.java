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
        initHardware();

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        setTime();

        while (opModeIsActive()) {
            telemetry.addData("State", state);
            telemetry.addData("Z", gyro.getIntegratedZValue());
            telemetry.addData("Sonic Dist", sonar.getUltrasonicLevel());
            telemetry.addData("Time: ", getTime());
            telemetry.addData("Color:", groundLeft.blue());

            switch (state) {
                case 0://start drive
                    setPower(0.25f);
                    state++;
                    break;

                case 1://drive 1 second
                    if(getTime() > 1.3){
                        setPower(0);
                        state++;
                    }
                    break;
                case 2://first turn
                    setPower(0.35f, -0.35f);
                    if(Math.abs(gyro.getIntegratedZValue()) > 44){
                        setPower(0);
                        state++;
                    }
                    break;
                case 3:
                    gyro.resetZAxisIntegrator();
                    state++;
                    break;
                case 4://drive until white line
                    driveGyro(0.3f);
                    if(isLeftOnLine()){
                        state ++;
                        setTime();
                    }
                    break;
                case 5:
                    if(getTime() > 0.3) {
                        setPower(0);
                        state++;
                    }
                    break;
                case 6:
                    setPower(0.35f, -0.35f);
                    if(Math.abs(gyro.getIntegratedZValue()) > 44){
                        setPower(0.35f);
                        state++;
                    }
                    break;
                case 7:
                    try {
                        telemetry.addData("STEP:", "potato 1");
                        if(sonar.getUltrasonicLevel() < 30){
                            setPower(0);
                            thrower.setPosition(1);
                            state++;
                        }
                    } catch (Exception e) {
                        telemetry.addData("ERROR:", "potato error 2");
                    }
                    break;
                case 8:
                    if(beacon.red() > beacon.blue()){
                        telemetry.addData("RED color: ", beacon.red());
                        beaconServo.setPosition(0);
                    } else {
                        telemetry.addData("BLUE color: ", beacon.blue());
                        beaconServo.setPosition(1);
                    }
                    beaconServo.setPosition(0);
                    try {
                        Thread.sleep(2000);
                    }catch (InterruptedException e){
                        telemetry.addData("ERROR", "POTATO Bread");
                    }
                    state++;
                    setPower(0.35f);
                    break;

                case 9:
                    if(sonar.getUltrasonicLevel() < 15){
                        setPower(0);
                    }
                    state++;
                    break;

//
//                case 6:
//                    setPower(0.25f, -0.25f);
//                    if(isRightOnLine()){
//                        setPower(-0.25f, 0.25f);
//                        state++;
//                    }
//                    break;
//                case 7:
//                    if(!isRightOnLine()){
//                        setPower(0);
//                        state=100;
//                    }
//                    break;
//
//                case 7: //turn onto white line
//                    if(isRightOnLine()){
//                        setPower(-0.2f, 0.2f);
//                    } else if(isLeftOnLine()){
//                        setPower(0.2f, -0.2f);
//                    } else {
//                        setPower(0.2f);
//                    }
//                    if(sonar.getUltrasonicLevel() < 10){
//                        setPower(0);
//                        state = 100;
//                    }
//                    break;


//
//                    setPower(0.25f, -0.25f);
//                    if((Math.abs(gyro.getIntegratedZValue()) > 44)) {
//                        setPower(0);
//                        state=100;
//                    }
//                    break;


//                //now facing wall
//
//               case 7://fixes throw distance and dumps climbers
//                    if(correctThrowDist()){ //also moves robot
//                      dumpClimbers();
//                       state++;
//                    }
//                    break;
//                case 8:
//                    if(correctBeaconDist()){
//                        setUpBeaconPress(); //reads on sets servo
//                        Thread.sleep(1500);//to allow servo to position its self
//                        state++;
//                        setPower(0.15f);
//                    }
//                    break;
//                case 10:
//                    if(hasPressedBeacon()){
//                        setPower(0);
//                        /*
//                        for (int i = 0; i < 2; i++){ //celebration
//                            leftWing.setPosition(1);
//                            rightWing.setPosition(1);
//                            leftWing.setPosition(0);
//                            rightWing.setPosition(0);
//                        }
//                        */
//                        state++;
//                    }
//                    break;
//                case 11: //potential turn left into wall
//                    setPower(-0.25f, 0.25f);
//                    if ((Math.abs(gyro.getIntegratedZValue()) > 90)){
//                        driveGyro(0.25f);
//                        if (getTime() >= 5){ //5 seconds
//                            setPower(0);
//                            state++;
//                        }
//                    }
                default:
                    break;
            }
        }
    }
}
