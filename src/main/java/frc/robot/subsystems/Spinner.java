package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldObject3D;
import frc.robot.util.ProjectileSimulator;
import frc.robot.util.TrajectorySolver;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class Spinner extends SubsystemBase {
  Pose2d HUB_POSE = new Pose2d(4.0, 4.1, new Rotation2d());
  Random rand = new Random();
  private List<FieldObject3D> allFuel = new ArrayList<>();
  // private boolean justShot;
  private double timeSinceLastShot = 0.0;
  // DoubleSupplier speed = () -> 8.5 + rand.nextGaussian() * 0.3;
  // DoubleSupplier yaw;
  Function<Double, Double> pitch;

  ProjectileSimulator sim = new ProjectileSimulator();
  TrajectorySolver trajectorySolver;

  Supplier<Pose2d> drivePose1;
  Supplier<Translation2d> velocity1;

  private final TalonFX spinner = new TalonFX(6, CANBus.roboRIO());
  private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
  // private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

  // private DoublePreferenceConstant spinnerSpeed =
  //     new DoublePreferenceConstant("Spinner/SpinnerSpeed", 0.0);

  // private MotionMagicPIDPreferenceConstants spinnerConfigConstants =
  //     new MotionMagicPIDPreferenceConstants("SpinnerMotors");

  public Spinner(Supplier<Pose2d> drivePose, Supplier<Translation2d> velocity) {
    drivePose1 = drivePose;
    velocity1 = velocity;
    trajectorySolver = new TrajectorySolver(drivePose, velocity);
    // yaw =
    //     () -> {
    //       return drivePose1.get().relativeTo(HUB_POSE).getTranslation().getAngle().getDegrees()
    //           + 180.0;
    //     };

    // configureTalons();

    for (int i = 0; i < 12; i++) {
      allFuel.add(
          new FieldObject3D(
              String.format("Field/Fuel%d", i), String.format("Field/Fuel%dTime", i)));
    }
  }

  public void periodic() {
    for (int i = 0; i < allFuel.size(); i++) {
      FieldObject3D fuel = allFuel.get(i);
      if (fuel.getCount() == 0) {
        if (fuel.hasTrajectory() && timeSinceLastShot > 2) {
          fuel.setPose();
          timeSinceLastShot = 0;
        } else if (fuel.hasTrajectory() && timeSinceLastShot <= 2) {

        } else {
          double speed = trajectorySolver.getShootSpeed();
          double angle = trajectorySolver.getAngle();
          double yaw = trajectorySolver.getYaw();
          fuel.setTrajectory(
              sim.simulate(drivePose1.get().getTranslation(), speed, yaw, angle, velocity1.get()));
        }
      } else {
        if (!fuel.setPose()) {
          double speed = trajectorySolver.getShootSpeed();
          double angle = trajectorySolver.getAngle();
          double yaw = trajectorySolver.getYaw();
          fuel.setTrajectory(
              sim.simulate(drivePose1.get().getTranslation(), speed, yaw, angle, velocity1.get()));
        }
      }
    }
    timeSinceLastShot++;
  }

  // private void configureTalons() {
  //   TalonFXConfiguration spinnerConfig = new TalonFXConfiguration();
  //   spinnerConfig.Slot0.kP = spinnerConfigConstants.getKP().getValue();
  //   spinnerConfig.Slot0.kI = spinnerConfigConstants.getKI().getValue();
  //   spinnerConfig.Slot0.kD = spinnerConfigConstants.getKD().getValue();
  //   spinnerConfig.Slot0.kV = spinnerConfigConstants.getKV().getValue();
  //   spinnerConfig.Slot0.kS = spinnerConfigConstants.getKS().getValue();
  //   spinnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
  //   spinner.getConfigurator().apply(spinnerConfig);
  // }

  private void setSpinnerSpeed(DoubleSupplier speed) {
    spinner.setControl(request.withVelocity(speed.getAsDouble()));
  }

  private void stopSpinnerMotors() {
    spinner.stopMotor();
  }

  // public Command runSpinner() {
  //   return new RunCommand(() -> setSpinnerSpeed(() -> spinnerSpeed.getValue()), this);
  // }

  public Command stopSpinner() {
    return new RunCommand(
        () -> {
          stopSpinnerMotors();
        },
        this);
  }
}
