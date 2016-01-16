package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.hardware.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.GyroSensor;

/**
 * IMPORTANT: do not move robot after hitting init
 */
public class BotHardware extends LinearOpMode {
    final int degreeError = 2;
    final int whiteLevel = 120;
    final int wallDistances = 5;

    DcMotor wfl, wbl, wfr, wbr;
    Servo thrower, leftWing, rightWing, beaconServo;
    ColorSensor groundLeft, groundRight, beacon;
    ModernRoboticsI2cGyro gyro;
    UltrasonicSensor sonar;

    double startTime;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    void initHardware() throws InterruptedException {
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
            thrower = hardwareMap.servo.get("thrower");
            leftWing = hardwareMap.servo.get("left_wing");
            rightWing = hardwareMap.servo.get("right_wing");
            beaconServo = hardwareMap.servo.get("beaconServo");
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
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "gyro sensor setup");
        }

        try {
            sonar = hardwareMap.ultrasonicSensor.get("sonar");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "sonar sensor setup");
        }

        while (gyro.isCalibrating()) {
            Thread.sleep(50);
        }

        waitForStart();

        startTime = this.getRuntime();
    }


    double getTime(){
        return getRuntime() - startTime;
    }

    void setPower (float power) {
        wfr.setPower(-power);
        wbr.setPower(-power);
        wfl.setPower(-power);
        wbl.setPower(-power);
    }

    void setPower(float left, float right) {
        wfr.setPower(-right);
        wbr.setPower(-right);
        wfl.setPower(-left);
        wbl.setPower(-left);
    }

    float scaleInput(float input) {
        telemetry.addData("[TEST]:", "calling scale input");
        input = Range.clip(input, -1, 1);
        if (input > 0)
            return (float)Math.pow(input, 4);
        return -(float)Math.pow(input, 4);
    }


    boolean turnDegrees(float power, int degrees) {
        gyro.resetZAxisIntegrator();
        if (degrees < 0) {
            setPower(power, -power);
        } else if (degrees > 0){
            setPower(-power, power);
        }
        if(Math.abs(gyro.getIntegratedZValue()) < Math.abs(degrees)) {
            setPower(0);
            return true;
        }
        return false;
    }

    void driveGyroStart(float power){
        setPower(0);
        gyro.resetZAxisIntegrator();
        setPower(power);
    }

    // will be in a loop (correction might be jerky, will test)
    void driveGyro(float power) {
        if (gyro.getIntegratedZValue() > degreeError)
            setPower(-power, power);
        else if (gyro.getIntegratedZValue() < -degreeError)
            setPower(power, -power);
        else
            setPower(power, power);
    }

    // will be in a loop (correction might be jerky, will test)
    void driveStraightLine(float power) {
        setPower(power);
        if (isOnLine(true))
            setPower(power, -power);
        else if (isOnLine(false))
            setPower(-power, power);
    }

    void turnGyroStart(float left, float right){
        setPower(0);
        gyro.resetZAxisIntegrator();
        setPower(left, right);
    }

    boolean turnGyro(double degree){
        return Math.abs(gyro.getIntegratedZValue()) < Math.abs(degree);
    }


    boolean isOnLine(boolean right) {
        return (groundRight.red() > whiteLevel && groundRight.blue() > whiteLevel && groundRight.green() > whiteLevel)
                || (groundLeft.red() > whiteLevel && groundLeft.blue() > whiteLevel && groundLeft.green() > whiteLevel);
    }

    void done(){
        telemetry.addData("Finished in:", getTime());
    }


    //drives to correct throw dist and returns if done
    boolean driveToDist(double dist){
        if(sonar.getUltrasonicLevel() == dist){
            setPower(0);
            return true;
        }

        if(sonar.getUltrasonicLevel() > dist){
            setPower(0.15f);
        } else if(sonar.getUltrasonicLevel() < dist){
            setPower(-0.15f);
        }
        return false;
    }


    double throwDist = 20;
    boolean correctThrowDist(){
        return driveToDist(throwDist);
    }

    double beaconDist = 5;
    boolean correctBeaconDist(){
        return driveToDist(beaconDist);
    }

    boolean hasPressedBeacon(){
        return sonar.getUltrasonicLevel() < 3;
    }

    void setUpBeaconPress(){
        if(leftSideRed())
            beaconServo.setPosition(0);
        else
            beaconServo.setPosition(1);
    }

    boolean leftSideRed(){
        return beacon.red() > beacon.blue();
    }

    void dumpClimbers() {
        thrower.setPosition(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
        }
        thrower.setPosition(0);
    }
}
