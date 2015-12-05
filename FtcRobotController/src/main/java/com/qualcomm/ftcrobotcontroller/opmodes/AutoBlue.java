package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class AutoBlue extends LinearOpMode {
    ColorSensor floor;
    // UltrasonicSensor ultraSensor;

    DcMotor wbr, wfr, wbl, wfl;
    Servo thrower, left_wing, right_wing;

    private double oldLeft = 0, oldRight = 0;

    boolean done = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitOneFullHardwareCycle();
        telemetry.addData("State", "Ready to Start");//dont press start until you see this
        waitForStart(); //waits fo start button to be pressed

        setPower(0.25f);
        while (!done) {
            if (isOnLine()) {
                setPower(0);
                done = true;
                sleep(100);
                setPower(0.5f, -0.5f); //turn right
                sleep(500);
                setPower(0.5f); //forward
                sleep(300);
                setPower(-.5f, .5f);//90 degree turn left
                sleep(1600);
                setPower(0.5f); //stop to throw in
                sleep(600);
                setPower(0);
                if(isOnLine()) {
                    thrower.setPosition(0.7);
                    sleep(2000);
                    thrower.setPosition(0);
                }
            }
            if(getRuntime() > 10){
                setPower(0);
                done = true;
            }
            telemetry.addData("Time", getRuntime());
            waitOneFullHardwareCycle();
        }
    }

    boolean reachedDist(int dist) {
        double distance = 5 * Math.PI;
        return wfl.getCurrentPosition() > distance;
    }

    void initHardware() {
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
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "thrower servo setup");
        }

        try {
            floor = hardwareMap.colorSensor.get("floor");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "floor color sensor setup");
        }
        /*
        try {
            ultraSensor = hardwareMap.ultrasonicSensor.get("ultraSensor");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "Ultrasonic sensor setup");
        }
        */
    }

    private boolean isOnLine() {
        if (!(floor.green() > 110 && floor.blue() > 110 && floor.red() > 110)) {
            telemetry.addData("Color:", "Not White");
            return false;
        } else {
            telemetry.addData("Color: ", "Is White");
            return true;
        }
    }

    void setPower(float left, float right) {
        // write the values to the motors
        wfr.setPower(-right);
        wbr.setPower(-right);
        wfl.setPower(-left);
        wbl.setPower(-left);
    }

    void setPower(float power) {
        // write the values to the motors
        wfr.setPower(-power);
        wbr.setPower(-power);
        wfl.setPower(-power);
        wbl.setPower(-power);
    }
}