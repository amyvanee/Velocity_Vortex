package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorTest extends LinearOpMode {
    ColorSensor floor;

    @Override
    public void runOpMode() throws InterruptedException {
        try {
            floor = hardwareMap.colorSensor.get("floor");
        } catch (Exception e) {
            telemetry.addData("Error", "Floor Color " + e.getStackTrace());
        }
        /*
        try {
            beacon = hardwareMap.colorSensor.get("beacon");
        } catch (Exception e) {
            telemetry.addData("Error", "Beacon Color " + e.getStackTrace());
        }
        */

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("Floor R", floor.red());
            telemetry.addData("Floor G", floor.green());
            telemetry.addData("Floor B", floor.blue());

            waitOneFullHardwareCycle();
        }
    }
}