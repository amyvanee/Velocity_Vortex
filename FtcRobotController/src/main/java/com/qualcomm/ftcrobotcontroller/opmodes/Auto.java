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
    //private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            initHardware();
        } catch (Exception e) {
            telemetry.addData("[ERROR]:", "hardware initialization error");
        }

        // waitOneFullHardwareCycle();
        // resetEncoders();

        waitOneFullHardwareCycle();
        telemetry.addData("State", "Ready to Start");//dont press start until you see this
        waitForStart(); //waits fo start button to be pressed

        setPower(0.05f);
        while (opModeIsActive()) {
            if (isOnLine()) {
                setPower(0);
                thrower.setPosition(1);
            }
            waitOneFullHardwareCycle();
        }
    }
//            switch (state) {
//                case 0:
//                    setPower(0.05f); // good speed for reading value
//                    state++;
//                    break;
//
//                case 1:
//                    if (isOnLine()) {
//                        setPower(0.0f);
//                        state++;
//                    }
//                    break;
//
//                case 2:
//                    servo.setPosition(1);
//                    sleep(1000);
//                    servo.setPosition(0);
//                    state++;
//                    break;
//
//                default:
//                    break;
//            }
//
//            telemetry.addData("State", state);
//
//            telemetry.addData("R", floor.red());
//            telemetry.addData("G", floor.green());
//            telemetry.addData("B", floor.blue());
//
//            waitOneFullHardwareCycle();


    void initHardware()
    {
        wfr = hardwareMap.dcMotor.get("wfr");
        wbr = hardwareMap.dcMotor.get("wbr");
        wfl = hardwareMap.dcMotor.get("wfl");
        wbl = hardwareMap.dcMotor.get("wbl");

        thrower = hardwareMap.servo.get("thrower");

        wbr.setDirection(DcMotor.Direction.REVERSE);
        wfr.setDirection(DcMotor.Direction.REVERSE);
    }

    private boolean isOnLine() {
        return floor.green() > 20 && floor.blue() > 20 && floor.red() > 20;
    }

    private void setPower(float i, float i2) {
        if (wbr != null)
            wbr.setPower(-i2);
        if (wfr != null)
            wfr.setPower(-i2);
        if (wbl != null)
            wbl.setPower(-i);
        if (wfl != null)
            wfl.setPower(-i);
    }

    private void setPower(float i) {
        if (wbr != null)
            wbr.setPower(-i);
        if (wfr != null)
            wfr.setPower(-i);
        if (wbl != null)
            wbl.setPower(-i);
        if (wfl != null)
            wfl.setPower(-i);
    }

    private void resetEncoders() {
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
}