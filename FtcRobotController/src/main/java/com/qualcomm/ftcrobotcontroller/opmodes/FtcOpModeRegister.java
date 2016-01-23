package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegister;

public class FtcOpModeRegister implements OpModeRegister {
  public void register(OpModeManager manager) {
    manager.register("TeleOp", TeleOp.class);
	manager.register("AutoRed", AutoRed.class);
    manager.register("AutoRedDelay", AutoRedDelay.class);
    manager.register("AutoBlue", AutoBlue.class);
    manager.register("AutoBlueDelay", AutoBlueDelay.class);
    manager.register("ColorTest", ColorTest.class);
    manager.register("TestOpMode", TestOpMode.class);
    manager.register("SonarCorrectionTest", SonarCorrectionTest.class);
    manager.register("Test", Test.class);
  }
}
