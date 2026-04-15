package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// can it deliver
// pose prediction perfection
// and survive the match?

public class Batman extends SubsystemBase {
  public double bestScore = 0;

  // private DoublePreferenceConstant rotationWeight =
  //    new DoublePreferenceConstant("Batman/RotationWeight", (18 / Math.PI)); // calculation

  @SuppressWarnings("unused")
  private boolean hasGlobalized = false;

  @SuppressWarnings("unused")
  private Pose3d lastPose = new Pose3d();

  private Pose3d currentPose = new Pose3d();

  @SuppressWarnings("unused")
  private boolean shouldUse = true;

  Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(-7.846),
          Units.inchesToMeters(8.992),
          Units.inchesToMeters(10.905),
          new Rotation3d(
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(0.0),
              Units.degreesToRadians(180.0)));

  // Transform3d ROBOT_TO_QUEST =
  //     new Transform3d(
  //         Units.inchesToMeters(0.0),
  //         Units.inchesToMeters(0.0),
  //         Units.inchesToMeters(0.0),
  //         new Rotation3d(0, 0, Units.degreesToRadians(-180.0)));

  private QuestNav quest = new QuestNav();

  public Batman() {}

  public void globalize(Pose3d globalPose) {
    resetPose(globalPose);
    hasGlobalized = true;
  }

  public Pose3d getPose() {
    return currentPose;
  }

  @AutoLogOutput(key = "Quest/CurrentPose")
  public Pose2d getPose2d() {
    return currentPose.toPose2d();
  }

  @AutoLogOutput(key = "Quest/RawQuestPose")
  public Pose2d getRawPose2d() {
    return currentPose.transformBy(ROBOT_TO_QUEST).toPose2d();
  }

  public boolean isTracking() {
    return quest.isTracking();
  }

  public boolean shouldUse() {
    return shouldUse;
  }

  public boolean isConnected() {
    return quest.isConnected();
  }

  public int getBatteryPercent() {
    return quest.getBatteryPercent().orElse(-1);
  }

  public void resetPose(Pose3d pose) {
    quest.setPose(pose.transformBy(ROBOT_TO_QUEST));
    Logger.recordOutput("Batman/ResetPose", pose.toPose2d());
    hasGlobalized = true;
  }

  public boolean hasGlobalized() {
    return hasGlobalized;
  }

  @Override
  public void periodic() {
    quest.commandPeriodic();
    Logger.recordOutput("Quest/Battery", getBatteryPercent());
    Logger.recordOutput("Quest/IsConnected", isConnected());

    if (!isConnected() || !isTracking()) {
      shouldUse = false;
      hasGlobalized = false;
    } else {
      shouldUse = true;
      PoseFrame[] poses = quest.getAllUnreadPoseFrames();
      if (poses.length > 0) {
        currentPose = poses[poses.length - 1].questPose3d();
        currentPose = currentPose.transformBy(ROBOT_TO_QUEST.inverse());
      }
    }
    Logger.recordOutput("Quest/ShouldUse", shouldUse);
  }

  public Command resetQuestPose(Supplier<Pose3d> pose) {
    return new InstantCommand(() -> globalize(pose.get()));
  }
}
