package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class AutoBlueDelay extends LinearOpMode {
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
        double timer = 0;
        sleep(10000);
        setPower(0.25f);
        while (!done) {
            timer = getRuntime();
            if (isOnLine()) {
                sleep(200);
                setPower(0);
                done = true;
                sleep(100);
                setPower(-0.5f, 0.5f); //turn right
                sleep(400);
                setPower(0.5f);
                sleep(500);
                setPower(0);
                if(isOnLine()) {
                    thrower.setPosition(0.7);
                    sleep(2000);
                    thrower.setPosition(0);
                }

            }
            if (getRuntime() - timer > 20) {
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

    /*
    private boolean isDroppable()
    {
        if(isOnLine())
        {
            if ultraSensor.1wwr
        }
    }
     */
}