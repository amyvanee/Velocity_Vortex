package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by vanee on 1/22/2016.
 */
public class SonarCorrectionTest extends BotHardware{
    @Override
    public void runOpMode() throws InterruptedException {
        short state = 0;
        short far, good, close;
        far = good = close = 0;

        initHardware();

        waitForStart();

        while (opModeIsActive())
        {
            switch (state) {
                case 0:
                    if(sonar.getUltrasonicLevel() < 23) {
                        far = good = 0;
                        close++;
                    } else if (sonar.getUltrasonicLevel() > 26) {
                        close = good = 0;
                        far++;
                    } else {
                        close = far = 0;
                        good++;
                    }

                    if (close == 2) {
                        setPower(-0.15f);
                        close = 0;
                    } else if (far == 2) {
                        setPower(0.15f);
                        far = 0;
                    } else if (good == 2) {
                        setPower(0);
                        thrower.setPosition(1);
                        state++;
                        close = far = good = 0;
                    }
                    break;
                 case 1:
                    setPower(0);
                    break;
            }
            waitOneFullHardwareCycle();
        }
    }
}
