package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class Auto extends LinearOpMode {
    ColorSensor floor;
    // UltrasonicSensor ultraSensor;

    DcMotor wbr, wfr, wbl, wfl;
    Servo thrower, left_wing, right_wing;

    private double oldLeft = 0, oldRight = 0;
    private boolean done = false;
    //private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitOneFullHardwareCycle();
        telemetry.addData("State", "Ready to Start");//dont press start until you see this
        waitForStart(); //waits fo start button to be pressed

        setPower(0.5f);
        while (!done) {
            if (isOnLine()) {
                sleep(100);
                setPower(0.5f, -0.5f);
                sleep(450);
                setPower(0.5f);
                sleep(150);
                setPower(-.5f, .5f); //90 degree turn left
                sleep(800);
                setPower(0);
                sleep(1000);

                /*
                setPower(0.5f);
                sleep(250);
                setPower(0);
                sleep(1000);
                 */

                thrower.setPosition(1);
                sleep(2000);
                thrower.setPosition(0);

                /*
                setPower(-0.5f, 0.5f)
                sleep(800);
                setPower(0);
                sleep(1000);
                setPower(0.5f);
                sleep(1000);
                setPower(0);
                 */

                done = true;
            }
        }
            waitOneFullHardwareCycle();


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
        }
        catch (Exception e) {
            telemetry.addData("[ERROR]:", "thrower servo setup");
        }

        try {
            floor = hardwareMap.colorSensor.get("floor");
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "floor color sensor setup");
        }
    }

    private boolean isOnLine() {
        return floor.green() > 20 && floor.blue() > 20 && floor.red() > 20;
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

    /* Got rid of this for testing

    private void resetEncoders()
    {
        oldLeft = wfl.getCurrentPosition();
        oldRight = wfr.getCurrentPosition();
        try {
            waitOneFullHardwareCycle();
        } catch (Exception e) {
            error(e);
        }
    }

    private double getLeft() {
        return Math.abs(wfl.getCurrentPosition() - oldLeft);
    }

    private double getRight() {
        return Math.abs(wfr.getCurrentPosition() - oldRight);
    }

    final double DIAMETER = 5; //5 inch wheels
    final double DIAGONAL_ROBOT = 24;//need to make more exact
    final double CIRCUMFERENCE = Math.PI * DIAMETER; // 15.7

    private void turn(float lp, float rp) {
        resetEncoders();
        try {
            waitOneFullHardwareCycle();
        } catch (InterruptedException e){

        }
        setPower(lp, rp);
    }

    private boolean testTurn(double degrees){
        double i = degrees * (((DIAGONAL_ROBOT * Math.PI) / CIRCUMFERENCE) * 280);
        if(Math.abs(getLeft()) >= i && Math.abs(getRight()) >= i) {
            setPower(0);
            return true;
        } else {
            return false;
        }
    }

    private void drive(float p) {
        resetEncoders();
        setPower(p);

    }

    private boolean testDrive(double distance){
        double i = (distance / CIRCUMFERENCE) * 280 / 360;
        if (Math.abs(getLeft()) >= i && Math.abs(getRight()) >= i) {
            setPower(0);
            return true;
        }
        return false;
    }

    private void done() {
        telemetry.addData("Finished in ", getRuntime());
    }

    private void error(Exception e) {
        telemetry.addData("Error", e.getStackTrace());
    }

    private void error(Exception e, String s) {
        telemetry.addData("Error", e.getStackTrace());
    }
    */
}