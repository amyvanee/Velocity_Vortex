package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class Auto extends LinearOpMode {
    ColorSensor adafruit, floor;
    UltrasonicSensor ultraSensor;

    DcMotor br, fr, bl, fl;
    Servo servo, beacon;

    private double oldLeft = 0, oldRight = 0;
    private int state = 0, secondState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            floor = hardwareMap.colorSensor.get("floor");
            ultraSensor = hardwareMap.ultrasonicSensor.get("ultraSensor");

            servo = hardwareMap.servo.get("thrower");

            beacon = hardwareMap.servo.get("beacon");

            adafruit = hardwareMap.colorSensor.get("color");
        } catch (Exception e) {
            error(e);
        }

        try {
            br = hardwareMap.dcMotor.get("wbr");
            br.setDirection(DcMotor.Direction.REVERSE);
            br.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        } catch (Exception e) {
            br = null;
            error(e, "WBR");
        }

        try {
            fr = hardwareMap.dcMotor.get("wfr");
            fr.setDirection(DcMotor.Direction.REVERSE);
            fr.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        } catch (Exception e) {
            fr = null;
            error(e, "WFR");
        }

        try {
            fl = hardwareMap.dcMotor.get("wfl");
            fl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        } catch (Exception e) {
            br = null;
            error(e, "WFL");
        }

        try {
            bl = hardwareMap.dcMotor.get("wbl");
            bl.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        } catch (Exception e) {
            bl = null;
            error(e, "WBL");
        }


        waitOneFullHardwareCycle();
        resetEncoders();

        bl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        br.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        fl.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        fr.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitOneFullHardwareCycle();
        telemetry.addData("State", "Ready to Start");//dont press start until you see this
        waitForStart(); //waits fo start button to be pressed

        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    setPower(0.05f); // good speed for reading value
                    state++;
                    break;

                case 1:
                    if (isOnLine()) {
                        setPower(0);
                        state++;
                    }
                    break;

                case 2:
                    turn(0.05f, -0.05f);
                    state++;
                    break;

                case 7:
                case 3:
                    if(testTurn(45)){
                        state++;
                    }
                    break;

                case 4:
                    drive(0.05f);
                    state++;
                    break;

                case 5:
                    if(testDrive(6)){
                        state++;
                    }
                    break;

                case 6:
                    turn(-0.05f, 0.05f);
                    state++;
                    break;

                case 8:
                    servo.setPosition(1);
                    sleep(1000);
                    servo.setPosition(0);
                    state++;
                    break;
                /*
                case 2:
                    if (ultraSensor.getUltrasonicLevel() > 10) {
                        switch (secondState) {
                            case 0:
                                setPower(0, 0.1f);
                                secondState++;
                                break;
                            case 1:
                                if (!isOnLine()) {
                                    setPower(0);
                                    secondState++;
                                }
                                break;
                            case 2:
                                setPower(0.1f, 0);
                                secondState++;
                                break;
                            case 3:
                                if (isOnLine()) {
                                    setPower(0);
                                    secondState = 0;
                                }
                                break;
                        }
                    } else {
                        setPower(0);
                        state++;
                    }
                    break;

                case 3:
                    if (getBeaconR() > getBeaconB()) {
                        beacon.setPosition(0.4);
                        sleep(1000);
                    } else {
                        beacon.setPosition(0.6);
                        sleep(1000);
                    }
                    setPower(0.1f);
                    state++;
                    break;

                case 4:
                    if (ultraSensor.getUltrasonicLevel() < 7) {
                        setPower(0);
                        state++;
                    }
                    break;

                case 5:
                    servo.setPosition(1);
                    sleep(1000);
                    servo.setPosition(0);
                    state++;
                    break;

                case 6:
                    done();
                    state++;
                    break;

                */
                default:
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Second State", secondState);

            telemetry.addData("R", floor.red());
            telemetry.addData("G", floor.green());
            telemetry.addData("B", floor.blue());

            telemetry.addData("BR", br.getCurrentPosition());
            telemetry.addData("FR", fr.getCurrentPosition());
            telemetry.addData("BL", bl.getCurrentPosition());
            telemetry.addData("FL", fl.getCurrentPosition());

            waitOneFullHardwareCycle();
        }

    }

    private boolean isOnLine() {
        return floor.green() > 20 && floor.blue() > 20 && floor.red() > 20;
    }

    private double getBeaconR() {
        return adafruit.red() * 255 / 800;
    }

    private double getBeaconB() {
        return adafruit.blue() * 255 / 800;
    }

    private void setPower(float i, float i2) {
        if (br != null)
            br.setPower(-i2);
        if (fr != null)
            fr.setPower(-i2);
        if (bl != null)
            bl.setPower(-i);
        if (fl != null)
            fl.setPower(-i);
    }

    private void setPower(float i) {
        if (br != null)
            br.setPower(-i);
        if (fr != null)
            fr.setPower(-i);
        if (bl != null)
            bl.setPower(-i);
        if (fl != null)
            fl.setPower(-i);
    }

    private void resetEncoders() {
        oldLeft = fl.getCurrentPosition();
        oldRight = fr.getCurrentPosition();
        try {
            waitOneFullHardwareCycle();
        } catch (Exception e) {
            error(e);
        }
    }

    private double getLeft() {
        return Math.abs(fl.getCurrentPosition() - oldLeft);
    }

    private double getRight() {
        return Math.abs(fr.getCurrentPosition() - oldRight);
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
        if(Math.abs(getLeft()) <= i && Math.abs(getRight()) <= i) {
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
        if (Math.abs(getLeft()) <= i && Math.abs(getRight()) <= i) {
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