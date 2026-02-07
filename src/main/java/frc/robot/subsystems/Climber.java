package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Climber extends SubsystemBase {

  private final TalonFX lift = new TalonFX(Constants.CLIMBER_LIFT, CANBus.roboRIO());
  private final TalonFX pivot = new TalonFX(Constants.CLIMBER_PIVOT, CANBus.roboRIO());
  private final CANrange canRange = new CANrange(Constants.CLIMBER_CANRANGE, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants liftPID =
      new MotionMagicPIDPreferenceConstants("Climber/LiftPID");
  private final MotionMagicPIDPreferenceConstants pivotPID =
      new MotionMagicPIDPreferenceConstants("Climber/PivotPID");

  private final MotionMagicVoltage liftMotionMagic = new MotionMagicVoltage(0);
  private final MotionMagicVoltage pivotMotionMagic = new MotionMagicVoltage(0);

  private final LinearFilter f = LinearFilter.singlePoleIIR(0.1, 0.02);
  private final Debouncer debouncer = new Debouncer(0.15);
  private boolean below = false;

  public Climber() {
    configureCANrange();
    configureMotors();
  }

  private void configureCANrange() {
    CANrangeConfiguration congifCaNrangeConfiguration = new CANrangeConfiguration(); // congif
    congifCaNrangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 50000;
    congifCaNrangeConfiguration.FovParams.FOVRangeX = 6.75;
    congifCaNrangeConfiguration.FovParams.FOVRangeY = 6.75;
    congifCaNrangeConfiguration.ProximityParams.ProximityThreshold = 0.1;
    congifCaNrangeConfiguration.ProximityParams.ProximityHysteresis = 0.02;
    canRange.getConfigurator().apply(congifCaNrangeConfiguration);
  }

  private void configureMotors() {
    TalonFXConfiguration liftConfig = new TalonFXConfiguration();
    liftConfig.Slot0.kP = liftPID.getKP().getValue();
    liftConfig.Slot0.kI = liftPID.getKI().getValue();
    liftConfig.Slot0.kD = liftPID.getKD().getValue();
    liftConfig.Slot0.kV = liftPID.getKV().getValue();
    liftConfig.Slot0.kS = liftPID.getKS().getValue();
    liftConfig.MotionMagic.MotionMagicCruiseVelocity = liftPID.getMaxVelocity().getValue();
    liftConfig.MotionMagic.MotionMagicAcceleration = liftPID.getMaxAcceleration().getValue();
    liftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO
    lift.getConfigurator().apply(liftConfig);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = pivotPID.getKP().getValue();
    pivotConfig.Slot0.kI = pivotPID.getKI().getValue();
    pivotConfig.Slot0.kD = pivotPID.getKD().getValue();
    pivotConfig.Slot0.kV = pivotPID.getKV().getValue();
    pivotConfig.Slot0.kS = pivotPID.getKS().getValue();
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotPID.getMaxVelocity().getValue();
    pivotConfig.MotionMagic.MotionMagicAcceleration = pivotPID.getMaxAcceleration().getValue();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO
    pivot.getConfigurator().apply(liftConfig);
  }

  public boolean isDetected() {
    double distance = f.calculate(canRange.getDistance().getValueAsDouble());
    if (debouncer.calculate(canRange.getSignalStrength().getValueAsDouble() > 50000) == false) {
      below = false;
      return false; // unknown state
    } else if (distance > 0.12) {
      below = false;
      return false; // too far
    } else if (distance < 0.08) {
      below = true;
      return true;
    } else if (distance < 0.12 && distance > 0.08) {
      if (below) {
        return true; // it was below .08 before
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  private void liftGotoPosition(double position) {
    lift.setControl(liftMotionMagic.withPosition(position));
  }

  private void pivotGotoPosition(double position) {
    pivot.setControl(pivotMotionMagic.withPosition(position));
  }

  public void periodic() {
    SmartDashboard.putNumber("Climber/CANrange/Distance", f.calculate(canRange.getDistance().getValueAsDouble()));
    SmartDashboard.putBoolean("Climber/CANrange/Detected", isDetected());
    SmartDashboard.putNumber("Climber/Lift/Position", lift.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Lift/Velocity", lift.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Lift/Voltage", lift.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Lift/Current", lift.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Pivot/Position", pivot.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Pivot/Velocity", pivot.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Pivot/Voltage", pivot.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Pivot/Current", pivot.getStatorCurrent().getValueAsDouble());
  }

  public Command liftGoto(double position) {
    return new RunCommand(() -> liftGotoPosition(position), this);
  }

  public Command pivotGoto(double position) {
    return new RunCommand(() -> pivotGotoPosition(position), this);
  }
}
