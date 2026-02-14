package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;

public class Climber extends SubsystemBase {

  private final TalonFX lift = new TalonFX(Constants.CLIMBER_LIFT, CANBus.roboRIO());
  private final TalonFX pivot = new TalonFX(Constants.CLIMBER_PIVOT, CANBus.roboRIO());
  private final CANrange canRange = new CANrange(Constants.CLIMBER_CANRANGE, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants liftPID =
      new MotionMagicPIDPreferenceConstants("Climber/LiftPID");
  private final MotionMagicPIDPreferenceConstants pivotPID =
      new MotionMagicPIDPreferenceConstants("Climber/PivotPID");
  private final DoublePreferenceConstant liftTestTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Test", 3.0);
  private final DoublePreferenceConstant liftGripTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Grip", 90.0);
  private final DoublePreferenceConstant liftTuckTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Tuck", 45.0);
  private final DoublePreferenceConstant liftDownTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Down", 0.0);
  private final DoublePreferenceConstant pivotTestTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Test", 0.0);
  private final DoublePreferenceConstant pivotLeftFlipTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/LeftFlip", 0.0);
  private final DoublePreferenceConstant pivotRightFlipTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/RightFlip", 0.0);
  private final DoublePreferenceConstant pivotSwitchTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Switch", 0.0);

  private final MotionMagicVoltage liftMotionMagic = new MotionMagicVoltage(0);
  private final MotionMagicVoltage pivotMotionMagic = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage liftMotionMagicSlow =
      new DynamicMotionMagicVoltage(0, 0, 0);
  private final VoltageOut liftVoltage = new VoltageOut(0.0);
  private final VoltageOut pivotVoltage = new VoltageOut(0.0);

  private final SysIdRoutine liftIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(2), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(this::setLiftVoltage, null, this));

  private final SysIdRoutine pivotIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(this::setPivotVoltage, null, this));

  private final LinearFilter f = LinearFilter.singlePoleIIR(0.1, 0.02);
  private final Debouncer debouncer = new Debouncer(0.15);
  private boolean below = false;

  public Climber() {
    configureCANrange();
    configureMotors();
    configureSmartDashboardButtons();
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
    pivotConfig.Slot0.kA = pivotPID.getKA().getValue();
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotPID.getMaxVelocity().getValue();
    pivotConfig.MotionMagic.MotionMagicAcceleration = pivotPID.getMaxAcceleration().getValue();
    pivotConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO
    pivot.getConfigurator().apply(pivotConfig);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Climber/Lift/calibrate", calibrate());
    SmartDashboard.putData("Climber/Flip Left", leftFlip());
    SmartDashboard.putData("Climber/Flip Right", rightFlip());
    SmartDashboard.putData("Climber/Go Grip", gotoGrip());
    SmartDashboard.putData("Climber/Go L1", gotoL1());
    SmartDashboard.putData("Climber/Go Ground", gotoGround());

    SmartDashboard.putData("Climber/Lift/Goto Target", liftGoto(() -> liftTestTarget.getValue()));
    SmartDashboard.putData(
        "Climber/Lift/Goto TargetClimb", liftGoto(() -> liftGripTarget.getValue()));
    SmartDashboard.putData(
        "Climber/Pivot/Goto Target", pivotGoto(() -> pivotTestTarget.getValue()));
    SmartDashboard.putData(
        "Climber/Pivot/Zero", new InstantCommand(() -> pivot.setPosition(0.0), this));
    SmartDashboard.putData(
        "Climber/Lift/Zero", new InstantCommand(() -> lift.setPosition(0.0), this));
    SmartDashboard.putData(
        "Climber/Lift/SysId/Quasistatic Forward", liftSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Climber/Lift/SysId/Quasistatic Reverse", liftSysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Climber/Pivot/SysId/Pivot Quasistatic Forward", pivotSysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Climber/Pivot/SysId/Pivot Quasistatic Reverse", pivotSysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData(
        "Climber/Lift/SysId/Dynamic Forward", liftSysIdDynamic(Direction.kForward));
    SmartDashboard.putData(
        "Climber/Lift/SysId/Dynamic Reverse", liftSysIdDynamic(Direction.kReverse));
    SmartDashboard.putData(
        "Climber/Pivot/SysId/Pivot Dynamic Forward", pivotSysIdDynamic(Direction.kForward));
    SmartDashboard.putData(
        "Climber/Pivot/SysId/Pivot Dynamic Reverse", pivotSysIdDynamic(Direction.kReverse));
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

  private void setLiftVoltage(Voltage volts) {
    lift.setControl(liftVoltage.withOutput(volts));
  }

  private void setPivotVoltage(Voltage volts) {
    pivot.setControl(pivotVoltage.withOutput(volts));
  }

  private void setCalibrate() {
    lift.setControl(new DutyCycleOut(-0.04));
  }

  private void flip(boolean flipRight) {
    if (lift.getPosition().getValueAsDouble() < (flipRight ? 50 : 65)) {
      pivotGotoPosition(
          flipRight ? pivotRightFlipTarget.getValue() : pivotLeftFlipTarget.getValue());
    }

    double position = Math.abs(pivot.getPosition().getValueAsDouble());

    if (position > pivotSwitchTarget.getValue()) {
      liftGotoPosition(liftGripTarget.getValue());
    } else {
      liftGotoPosition(liftTuckTarget.getValue());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber(
        "Climber/CANrange/Distance", f.calculate(canRange.getDistance().getValueAsDouble()));
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

  public Command liftGoto(DoubleSupplier target) {
    return new RunCommand(() -> liftGotoPosition(target.getAsDouble()), this);
  }

  public Command pivotGoto(DoubleSupplier position) {
    return new RunCommand(() -> pivotGotoPosition(position.getAsDouble()), this);
  }

  public Command liftSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return liftIdRoutine.quasistatic(direction);
  }

  public Command pivotSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return pivotIdRoutine.quasistatic(direction);
  }

  public Command liftSysIdDynamic(SysIdRoutine.Direction direction) {
    return liftIdRoutine.dynamic(direction);
  }

  public Command pivotSysIdDynamic(SysIdRoutine.Direction direction) {
    return pivotIdRoutine.dynamic(direction);
  }

  public Command calibrate() {
    return new SequentialCommandGroup(
            new RunCommand(() -> setCalibrate(), this)
                .until(() -> lift.getStatorCurrent().getValueAsDouble() > 15.0),
            new InstantCommand(() -> lift.setPosition(0.0), this))
        .andThen(new RunCommand(() -> lift.setControl(new DutyCycleOut(0.0)), this));
  }

  public Command gotoGrip() {
    return new RunCommand(() -> liftGotoPosition(liftGripTarget.getValue()), this);
  }

  public Command gotoL1() {
    return new RunCommand(() -> liftGotoPosition(liftTuckTarget.getValue()), this);
  }

  public Command gotoGround() {
    return new RunCommand(() -> liftGotoPosition(liftDownTarget.getValue()), this);
  }

  public Command rightFlip() {
    return new RunCommand(() -> flip(true), this);
  }

  public Command leftFlip() {
    return new RunCommand(() -> flip(false), this);
  }
}
