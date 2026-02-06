package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class Batman extends SubsystemBase {
  @SuppressWarnings("unused")
  private boolean hasGlobalized = false;

  @SuppressWarnings("unused")
  private Pose3d lastPose = new Pose3d();

  private Pose3d currentPose = new Pose3d();

  @SuppressWarnings("unused")
  private boolean shouldUse = true;

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
    quest.setPose(pose);
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
}
