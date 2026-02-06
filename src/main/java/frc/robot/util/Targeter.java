package frc.robot.util;

import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class Targeter {
    private static Supplier<Pose3d> poseSupplier;
    private static Supplier<Translation2d> velocitySupplier;
    //TODO Replace with interpolated value
    private static Function<Double, Double> interpolationHoodAngle = (Double distance) -> 0.0;

    public static void setPoseAndVelocity(Supplier<Pose3d> pose, Supplier<Translation2d> velocity) {
        poseSupplier = pose;
        velocitySupplier = velocity;
    }

    public static double getTurretAngle(Pose3d pose) {
        return 0.0;
    }

    public static double getHoodAngle(Pose3d pose) {
        return interpolationHoodAngle.apply(getDistanceToHub(pose));
    }

    public static double getDistanceToHub(Pose3d pose) {
        return pose.relativeTo(Constants.HUB_POSE).getTranslation().getNorm();
    }

    

}
