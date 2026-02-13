package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.util.ProjectileSimulator.Pose3dTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class FieldObject3D {
  private final StructPublisher<Pose3d> posePublisher;
  private final DoublePublisher timePub;
  public int count = 0;
  public boolean hasSetTime = false;
  List<Pose3dTime> trajectory = Collections.emptyList();
  List<Double> allTimes = new ArrayList<>();

  public FieldObject3D(String topicName, String timeName) {
    posePublisher =
        NetworkTableInstance.getDefault().getStructTopic(topicName, Pose3d.struct).publish();
    timePub = NetworkTableInstance.getDefault().getDoubleTopic(timeName).publish();
  }

  public void setTrajectory(List<Pose3dTime> poses) {
    trajectory = poses;
  }

  public boolean setPose() {
    if (trajectory.isEmpty() || count >= trajectory.size()) {
      count = 0;
      hasSetTime = false;
      // System.out.println(allTimes.toString());
      return false;
    }
    Pose3dTime current = trajectory.get(count);
    if (!hasSetTime) {
      allTimes.add(trajectory.get(trajectory.size() - 1).getTime());
      hasSetTime = true;
    }
    // double time = trajectory.get(trajectory.size() - 1).getTime();
    posePublisher.set(current.toPose3d());
    timePub.set(current.getTime());
    count++;
    return true;
  }

  public int getCount() {
    return count;
  }

  public boolean hasTrajectory() {
    return !trajectory.isEmpty();
  }
}
