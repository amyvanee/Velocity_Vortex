package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorTest extends BotHardware {
    ColorSensor floor;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("sonar:", sonar.getUltrasonicLevel());
            telemetry.addData("Beacon R:", beacon.red());
            telemetry.addData("Beacon B:", beacon.blue());
            telemetry.addData("Beacon G:", beacon.green());
            telemetry.addData("Floor R:", floor.red());
            telemetry.addData("Floor B:", floor.blue());
            telemetry.addData("Floor G:", floor.green());
            waitOneFullHardwareCycle();
        }
    }
}
