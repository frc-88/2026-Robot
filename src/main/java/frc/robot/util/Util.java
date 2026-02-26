package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Util {

  public static boolean logif() {
    return Constants.MODE == Constants.ModeCode.DEVELOPMENT;
  }

  public static boolean weAreRed() { // copied from drive
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }
}
