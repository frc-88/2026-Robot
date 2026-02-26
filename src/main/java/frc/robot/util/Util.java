package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Util {

  public static boolean logif() {
    return Constants.MODE == Constants.ModeCode.DEVELOPMENT;
  }

  public static boolean weAreRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  private static Pose2d flipPose(Pose2d pose) {
    return pose.relativeTo(
        new Pose2d(
            Constants.FIELD_LENGTH,
            Constants.FIELD_WIDTH,
            new Rotation2d(Units.degreesToRadians(180))));
  }

  public static Pose2d flipIfRed(Pose2d pose) {
    return weAreRed() ? flipPose(pose) : pose;
  }

  public static Translation2d flipIfRed(Translation2d translation) {
    return flipIfRed(new Pose2d(translation, new Rotation2d())).getTranslation();
  }
}
