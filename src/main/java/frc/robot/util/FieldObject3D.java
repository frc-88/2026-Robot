package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/** Publishes a single 3D field object (ex: ball, note, cube) to AdvantageScope. */
public class FieldObject3D {
  private final StructPublisher<Pose3d> posePublisher;

  /**
   * @param topicName NetworkTables topic name (ex: "Field/Ball")
   */
  public FieldObject3D(String topicName) {
    posePublisher =
        NetworkTableInstance.getDefault().getStructTopic(topicName, Pose3d.struct).publish();
  }

  /**
   * Set the object's pose on the field.
   *
   * @param x Field X (meters)
   * @param y Field Y (meters)
   * @param z Field Z (meters)
   * @param rotation Object rotation
   */
  public void setPose(double x, double y, double z, Rotation3d rotation) {
    posePublisher.set(new Pose3d(x, y, z, rotation));
  }
}
