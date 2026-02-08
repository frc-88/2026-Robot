package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Batman extends SubsystemBase {
  public double bestScore = 0;
  public Pose2d drivePose;
  public Pose3d visionPose;
  private DoublePreferenceConstant rotationWeight =
      new DoublePreferenceConstant("Batman/RotationWeight", (18 / Math.PI)); // calculation

  @SuppressWarnings("unused")
  private boolean hasGlobalized = false;

  @SuppressWarnings("unused")
  private Pose3d lastPose = new Pose3d();

  private Pose3d currentPose = new Pose3d();

  @SuppressWarnings("unused")
  private boolean shouldUse = true;

  Transform3d ROBOT_TO_QUEST =
      new Transform3d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          new Rotation3d(0, 0, 0));

  private QuestNav quest = new QuestNav();

  public void globalize(Pose3d globalPose) {
    resetPose(globalPose);
    hasGlobalized = true;
  }

  public Pose3d getPose() {
    return currentPose;
  }

  public boolean isTracking() {
    return quest.isTracking();
  }

  public boolean isConnected() {
    return quest.isConnected();
  }

  public int getBatteryPercent() {
    return quest.getBatteryPercent().orElse(-1);
  }

  public void resetPose(Pose3d pose) {
    quest.setPose(pose.transformBy(ROBOT_TO_QUEST));
  }

  @Override
  public void periodic() {
    quest.commandPeriodic();
    SmartDashboard.putNumber("Quest/Battery", getBatteryPercent());
    SmartDashboard.putBoolean("Quest/isConnected", isConnected());

    if (!isConnected() || !isTracking()) {
      shouldUse = false;
      hasGlobalized = false;
    } else {
      shouldUse = true;
      PoseFrame[] poses = quest.getAllUnreadPoseFrames();
      if (poses.length > 0) {
        currentPose = poses[poses.length - 1].questPose3d();
        lastPose = currentPose;
      }
    }
  }

  public void checkPose(Pose2d newPose, double linearStddev, double angularStddev) {
    Pose2d newPoseDiff = newPose.relativeTo(drivePose);
    double score = linearStddev + rotationWeight.getValue() * angularStddev;
    score =
        score
            + newPoseDiff.getX()
            + newPoseDiff.getY()
            + rotationWeight.getValue() * newPoseDiff.getRotation().getRadians();
    if (bestScore == 0 || score < bestScore) {
      bestScore = score;
      resetPose(visionPose);
    }
  }

  public Command resetQuestPose(Pose3d pose) {
    return new InstantCommand(() -> resetPose(pose));
  }
}
