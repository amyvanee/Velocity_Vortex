package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

public class AutoBlue extends BotHardware {
    private int state = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive()){
            switch (state) {
                case 0:
                    driveDistance(0.15f, 6);
                    state++;
                    break;
                case 1:
                    turnDegrees(0.15f, 45, true);
                    state++;
                    break;
                case 2:
                    driveStraightGryo(0.15f);
                    if (isOnLine(true))
                        state++;
                    break;
                case 3:
                    turnDegrees(0.15f, 45, false);
                    state++;
                    break;
                case 4:
                    driveStraightLine(0.15f);
                    if (isAtWall())
                        state++;
                    break;
                case 5:
                    dumpClimbers();
                    state++;
                    break;
                case 6:
                    if (!isBeaconRed())
                        driveDistance(0.15f, 2);
                    state++;
                case 7:
                    // done
            }
        }
    }
}