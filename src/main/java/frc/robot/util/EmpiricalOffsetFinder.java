package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EmpiricalOffsetFinder extends SubsystemBase{
    
    private Supplier<Pose2d> pose;
    private BooleanSupplier usable;
    private Rotation2d firstRotation;

    private List<Translation2d> poses = new ArrayList<Translation2d>();

    public EmpiricalOffsetFinder(BooleanSupplier shouldUse, Supplier<Pose2d> poseSupplier) {
        pose = poseSupplier;
        usable = shouldUse;
        SmartDashboard.putData("OffsetFinder/Fit", fitThePoints());
    }

    public void periodic() {
        if (poses.size() == 0) {
            firstRotation = pose.get().getRotation();
            Logger.recordOutput("OffsetFinder/FirstRotation", firstRotation);
        }
        if (usable.getAsBoolean()) {
            poses.add(pose.get().getTranslation());
        }
    }

    public record CircleFitResult(
    Translation2d center,
    double radius
) {}

    @AutoLogOutput
    private CircleFitResult fit(List<Translation2d> points) {
        int n = points.size();
        if (n<3) {
            System.out.println("STOPDude");
            return new CircleFitResult(new Translation2d(0.0,0.0),0.0);
        }

        double sumX = 0, sumY = 0;
        double sumX2 = 0, sumY2 = 0;
        double sumXY = 0;
        double sumX3 = 0, sumY3 = 0;
        double sumX1Y2 = 0, sumX2Y1 = 0;


        for (Translation2d p : points) {
            double x = p.getX();
            double y = p.getY();
            double x2 = x * x;
            double y2 = y * y;

            sumX += x;
            sumY += y;
            sumX2 += x2;
            sumY2 += y2;
            sumXY += x * y;
            sumX3 += x2 * x;
            sumY3 += y2 * y;
            sumX1Y2 += x * y2;
            sumX2Y1 += x2 * y;
        }

        double C = n * sumX2 - sumX * sumX;
        double D = n * sumXY - sumX * sumY;
        double E = n * (sumX3 + sumX1Y2) - (sumX2 + sumY2) * sumX;
        double G = n * sumY2 - sumY * sumY;
        double H = n * (sumX2Y1 + sumY3) - (sumX2 + sumY2) * sumY;

        double denominator = 2 * (C * G - D * D);
        if (Math.abs(denominator) < 1e-12) {
            throw new RuntimeException("Points are collinear or poorly conditioned");
        }

        double a = (G * E - D * H) / denominator;
        double b = (C * H - D * E) / denominator;

        double cx = a;
        double cy = b;

        double radius = 0;
        for (Translation2d p : points) {
            radius += p.getDistance(new Translation2d(cx, cy));
        }
        radius /= n;

        return new CircleFitResult(new Translation2d(cx, cy), radius);
    }

    public Command fitThePoints() {
        return new InstantCommand(() -> fit(poses));
    }
}
