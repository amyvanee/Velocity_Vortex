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

    /*
    final private double wheelCircumference = 5 * Math.PI;
    final private short degreeError = 2;
    final private short whiteLevel = 25;
    final private short wallDistances = 5;

    private short state = 0;
    private short sonar_times = 0; // needed to compensate for sonar 0

    private DcMotor wfl, wbl, wfr, wbr, arm;
    private ColorSensor groundLeft, groundRight, beacon;
    private ModernRoboticsI2cGyro gyro;
    private UltrasonicSensor sonar;

    private short rounds = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()) {
            while (rounds < 5){
                leftWing.setPosition(1);
                rightWing.setPosition(1);

                turnDegrees(0.1f, 360);
                while (gyro.getIntegratedZValue() < 360){
                    leftWing.setPosition(0.5);
                    rightWing.setPosition(0.5);
                    Thread.sleep(10);
                    leftWing.setPosition(1);
                    rightWing.setPosition(1);
                }
                rounds++;
            }
        }
    }

    void initHardware() {
        try {
            arm = hardwareMap.dcMotor.get("arm");
            arm.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "arm motor and controller setup");
        }

        try {
            wfr = hardwareMap.dcMotor.get("wfr");
            wbr = hardwareMap.dcMotor.get("wbr");
            wfl = hardwareMap.dcMotor.get("wfl");
            wbl = hardwareMap.dcMotor.get("wbl");

            wbr.setDirection(DcMotor.Direction.REVERSE);
            wfr.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "driving wheels setup");
        }

        try {
            thrower = hardwareMap.servo.get("thrower");
            leftWing = hardwareMap.servo.get("left_wing");
            rightWing = hardwareMap.servo.get("right_wing");
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "servo setup");
        }

        try {
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "gyro sensor setup");
        }

        try {
            gyro.calibrate();
            while (gyro.isCalibrating()) {
                sleep(50);
            }
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "calibrating issue");
        }

        try {
            groundLeft = hardwareMap.colorSensor.get("groundLeft");
            groundRight = hardwareMap.colorSensor.get("groundRight");
            beacon = hardwareMap.colorSensor.get("beacon");
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "color sensor setup");
        }

        try {
            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
            gyro.calibrate();
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "gyro sensor setup");
        }

        try {
            sonar = hardwareMap.ultrasonicSensor.get("sonar");
        } catch (Exception e) {
            telemetry.addData("[ERROR]", "sonar sensor setup");
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
        input = Range.clip(input, -1, 1);
        if (input > 0)
            return (float)Math.pow(input, 4);
        return -(float)Math.pow(input, 4);
    }

    void turnDegrees(float power, int degrees) {
        gyro.resetZAxisIntegrator();
        if (degrees > 0) {
            setPower(power, -power);
        } else {
            setPower(-power, power);
        }
        while (Math.abs(gyro.getIntegratedZValue()) < Math.abs(degrees)) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                telemetry.addData("[ERROR]", "distance wait");
            }
        }
        setPower(0f);
    }

    // will be in a loop (correction might be jerky, will test)
    void driveStraightGyro(float power) {
        setPower(power);
        double angle = gyro.getIntegratedZValue();
        if (angle > degreeError) {
            setPower(0.2f, -0.2f);
        }
        else if (angle < -degreeError) {
            setPower(-0.2f, 0.2f);
        }
        else {
            setPower(power, power);
        }
    }

    // will be in a loop (correction might be jerky, will test)
    void driveStraightLine(float power) {
        setPower(power);
        if (isOnLine(true))
            setPower(power, -power);
        else if (isOnLine(false))
            setPower(-power, power);
    }

    boolean isOnLine(boolean right) {
        if (right) {
            telemetry.addData("Right Color", "R: " + groundRight.red() + " G: " + groundRight.green() + " B: " + groundRight.blue());
            return groundRight.red() > whiteLevel && groundRight.blue() > whiteLevel && groundRight.green() > whiteLevel;
        } else {
            telemetry.addData("Left Color", "R: " + groundLeft.red() + " G: " + groundLeft.green() + " B: " + groundLeft.blue());
            return groundLeft.red() > whiteLevel && groundLeft.blue() > whiteLevel && groundLeft.green() > whiteLevel;
        }
    }

    boolean isAtWall() {
        return sonar.getUltrasonicLevel() < wallDistances;
    }

    void dumpClimbers() {
        thrower.setPosition(1);
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            // nothing
        }
        thrower.setPosition(0);
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

    // subject to change
    boolean isBeaconRed() {
        return beacon.red() > 100 && beacon.blue() < 90;
    }

    void driveToDistance(float power, int distance){

        if (sonar_times < 2) {
            driveStraightLine(power); //no longer driveStraightGyro
            double level = sonar.getUltrasonicLevel();
            telemetry.addData("Sonar", level);
            if (level < distance) {
                sonar_times++;
            } else {
                sonar_times = 0;
            }
        }
        if (sonar_times == 2) {
            setPower(0, 0);
            sonar_times++;
        }
    }
    */
}
