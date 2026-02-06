package frc.robot.util;

import frc.robot.Constants;

public class Util {

  public static boolean logif() {
    return Constants.MODE == Constants.ModeCode.DEVELOPMENT;
  }
}
