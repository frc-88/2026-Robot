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
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.Util;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  private final CANBus canbus = new CANBus("Drivetrain");
  private final TalonFX lift = new TalonFX(Constants.CLIMBER_LIFT, canbus);
  private final TalonFX pivot = new TalonFX(Constants.CLIMBER_PIVOT, canbus);
  private final CANrange canRange = new CANrange(Constants.CLIMBER_CANRANGE, canbus);
  private final Pigeon2 pigeon = new Pigeon2(0, canbus);
  // PowerDistribution pdh = new PowerDistribution();
  // private final Pigeon2 basePigeon = new Pigeon2(Constants.BASE_PIGEON, CANBus.roboRIO());

  private final MotionMagicPIDPreferenceConstants liftPID =
      new MotionMagicPIDPreferenceConstants(
          "Climber/LiftPID", 88., 400., 1000., 40., 0., 0., 0.11047, 0.23, 0.);
  private final MotionMagicPIDPreferenceConstants pivotPID =
      new MotionMagicPIDPreferenceConstants(
          "Climber/PivotPID", 500., 10000., 0., 10., 0., 0., 0.11048, 0.0, 0.0195);
  private final DoublePreferenceConstant liftTestTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Test", 0.0);
  private final DoublePreferenceConstant liftGripTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Grip", 90.4);
  private final DoublePreferenceConstant liftFlipGripTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/FlipGrip", 91.9);
  private final DoublePreferenceConstant liftChinStrapTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/ChinStrap", 89.81);
  private final DoublePreferenceConstant liftTuckTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Tuck", 25.48);
  private final DoublePreferenceConstant liftDownTarget =
      new DoublePreferenceConstant("Climber/Lift/Target/Down", 1.0);
  private final DoublePreferenceConstant pivotTestTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Test", 7.4);
  private final DoublePreferenceConstant pivotGripTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Grip", -10.0);
  private final DoublePreferenceConstant pivotStowTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Stow", 7.4);
  private final DoublePreferenceConstant pivotFirstTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/FirstTarget", 150.0);
  private final DoublePreferenceConstant pivotL1Target =
      new DoublePreferenceConstant("Climber/Pivot/Target/L1", 0.0);
  private final DoublePreferenceConstant pivotChinStrapTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/ChinStrap", -10.0);
  private final DoublePreferenceConstant pivotFlipTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Flip", 166.5);
  private final DoublePreferenceConstant pivotFlipDelay =
      new DoublePreferenceConstant("Climber/Pivot/Target/Delay", 30.0);
  private final DoublePreferenceConstant pivotSwitchTarget =
      new DoublePreferenceConstant("Climber/Pivot/Target/Switch", 90.0);

  private final MotionMagicVoltage liftMotionMagic = new MotionMagicVoltage(0);
  private final DynamicMotionMagicVoltage dynamic =
      new DynamicMotionMagicVoltage(0.0, 10.0, -200.0);
  private final MotionMagicVoltage pivotMotionMagic =
      new MotionMagicVoltage(0).withFeedForward(3.0);
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
  private double lastTime = 0.0;
  private double currentTime = 1.0;

  public Climber() {
    configureCANrange();
    configureMotors();
    configureSmartDashboardButtons();

    // liftTestTarget.addChangeHandler(
    //     (Double abcdefghijklmnopqrstuvwxyz1234567890) -> configureMotors());
    // pivotTestTarget.addChangeHandler(
    //     (Double abcdefghijklmnopqrstuvwxyz1234567890) -> configureMotors());
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
    liftConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    liftConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    liftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO
    lift.getConfigurator().apply(liftConfig);

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.Slot0.kP = pivotPID.getKP().getValue();
    pivotConfig.Slot0.kI = pivotPID.getKI().getValue();
    pivotConfig.Slot0.kD = pivotPID.getKD().getValue();
    pivotConfig.Slot0.kV = pivotPID.getKV().getValue();
    pivotConfig.Slot0.kS = pivotPID.getKS().getValue();
    pivotConfig.Slot0.kA = pivotPID.getKA().getValue();
    // pivotConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotConfig.Slot0.kG = 3.0;
    // pivotConfig.Slot0.GravityArmPositionOffset = -0.25;
    // pivotConfig.Feedback.SensorToMechanismRatio =
    // Constants.CLIMBER_PIVOT_ROTATIONS_TO_ROBOT_ROTATIONS;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    pivotConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity = pivotPID.getMaxVelocity().getValue();
    pivotConfig.MotionMagic.MotionMagicAcceleration = pivotPID.getMaxAcceleration().getValue();
    pivotConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // TODO

    pivot.getConfigurator().apply(pivotConfig);
    pivot.setNeutralMode(NeutralModeValue.Brake);
    lift.setNeutralMode(NeutralModeValue.Brake);

    pivot.setPosition(7.4);
  }

  private void configureSmartDashboardButtons() {
    SmartDashboard.putData("Climber/Lift/Lift Calibrate", calibrate());
    SmartDashboard.putData("Climber/Flip", flipCommand());
    SmartDashboard.putData("Climber/Go Grip", goToGrip());
    SmartDashboard.putData("Climber/Go L1", gotoL1());
    SmartDashboard.putData("Climber/Go Stow", gotoStow());
    SmartDashboard.putData("Climber/Go Chinstrap", goToChinStrap());
    SmartDashboard.putData("Climber/Go Pole", getOnPole());
    SmartDashboard.putData("Climber/JustPivotL1", pivotGotToL1());

    SmartDashboard.putData(
        "Climber/Lift/Lift Go Target", liftGoto(() -> liftTestTarget.getValue()));
    SmartDashboard.putData(
        "Climber/Pivot/Pivot Go Target", pivotGoto(() -> pivotTestTarget.getValue()));
    SmartDashboard.putData(
        "Climber/Pivot/ZeroPivot",
        new InstantCommand(() -> pivot.setPosition(7.4), this).ignoringDisable(true));
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

  @AutoLogOutput
  public boolean isHealthy() {
    return lift.isConnected() && lift.isAlive() && pivot.isConnected() && pivot.isAlive();
  }

  @AutoLogOutput
  public boolean isPartiallyOnPole() {
    return debouncer.calculate(getDistance() < 0.3 && getDistance() > 0.215);
  }

  @AutoLogOutput
  public boolean isFullyOnPole() { // TODO SIGNAL STRENGTH
    return getDistance() < 0.215
        && lift.getPosition().getValueAsDouble() > liftGripTarget.getValue() - 0.2;
  }

  public boolean isAtTuck() {
    return Math.abs(lift.getPosition().getValueAsDouble() - liftTuckTarget.getValue()) < 1.0;
  }

  private void stop() {
    lift.setControl(new DutyCycleOut(0));
    pivot.setControl(new DutyCycleOut(0));
  }

  private void liftGotoPosition(double position) {
    Logger.recordOutput("Climber/LiftPositionSetpoint", position);
    lift.setControl(liftMotionMagic.withPosition(position));
  }

  private void pivotGotoPosition(double position) {
    // if (pivot.getPosition().getValueAsDouble() >= 40.0) {
    //   Logger.recordOutput("IsDynamic", true);
    //   pivot.setControl(dynamic.withPosition(position)); // * sin
    // } else {
    // Logger.recordOutput("IsDynamic", false);
    pivot.setControl(pivotMotionMagic.withPosition(position));
    // }
    // pitch
    // pivot.setControl(new DutyCycleOut(1.0));
  }

  @AutoLogOutput
  public double getPivotVelocity() {
    return pivot.getVelocity().getValueAsDouble();
  }

  private void pivotGotoPositionMotion(boolean flipRight) {
    pivot.setControl(
        pivotMotionMagic.withPosition(
            Math.abs(
                pivot.getPosition().getValueAsDouble()
                    - (Math.signum(pigeon.getRoll().getValueAsDouble())
                        * (((180.0 - Math.abs(pigeon.getRoll().getValueAsDouble())) / 360.0)
                            * 592)))));
  }

  private void setLiftVoltage(Voltage volts) {
    lift.setControl(liftVoltage.withOutput(volts));
  }

  private void setPivotVoltage(Voltage volts) {
    pivot.setControl(pivotVoltage.withOutput(volts));
  }

  private void setCalibrate() {
    lift.setControl(new DutyCycleOut(-0.08));
  }

  private void flip() {
    if (lift.getPosition().getValueAsDouble() < (pivotFlipDelay.getValue())) {
      // if (!pivotAtPosition(pivotFirstTarget.getValue())) {
      //   pivotGotoPosition(pivotFirstTarget.getValue());
      // } else {
      //   pivotGotoPosition(pivotFlipTarget.getValue());
      // }
      pivotGotoPosition(pivotFlipTarget.getValue());
    }

    double position = Math.abs(pivot.getPosition().getValueAsDouble());

    if (position > pivotSwitchTarget.getValue()) {
      liftGotoPosition(liftFlipGripTarget.getValue());
    } else {
      liftGotoPosition(liftTuckTarget.getValue());
    }
  }

  private boolean pivotAtPosition(double target) {
    return Math.abs(pivot.getPosition().getValueAsDouble() - target) < 2.0;
  }

  // private void flipMotion(boolean flipRight) {
  //   if (lift.getPosition().getValueAsDouble() < (pivotFlipDelay.getValue())) {
  //     pivotGotoPosition(pivotFlipTarget.getValue());
  //   }

  //   double position = Math.abs(pivot.getPosition().getValueAsDouble());

  //   if (position > pivotSwitchTarget.getValue()) {
  //     liftGotoPosition(liftGripTarget.getValue());
  //   } else {
  //     liftGotoPosition(liftTuckTarget.getValue());
  //   }
  // }

  public double getDistance() {
    if (canRange.getSignalStrength().getValueAsDouble() > 2000.0) {
      return f.calculate(canRange.getDistance().getValueAsDouble());
    } else {
      return 10000.0;
    }
  }

  public double getLiftPosition() {
    return lift.getPosition().getValueAsDouble();
  }

  private void liftGetOnPole() {
    if (isPartiallyOnPole()
        || (lift.getPosition().getValueAsDouble() > liftGripTarget.getValue() - 0.5)) {
      pivotGotoPosition(pivotGripTarget.getValue());
      liftGotoPosition(liftGripTarget.getValue());
    } else {
      liftGotoPosition(liftChinStrapTarget.getValue());
      pivotGotoPosition(pivotChinStrapTarget.getValue());
    }
  }

  public void periodic() {
    currentTime = Timer.getFPGATimestamp();
    if (Util.logif()) {
      SmartDashboard.putNumber(
          "Climber/CANrange/Distance", f.calculate(canRange.getDistance().getValueAsDouble()));
      SmartDashboard.putNumber("Climber/Lift/Position", lift.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Lift/Velocity", lift.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Lift/Voltage", lift.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Lift/Current", lift.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Pivot/Position", pivot.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Pivot/Velocity", pivot.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Climber/Pivot/Voltage", pivot.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber(
          "Climber/Pivot/Current", pivot.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber(
          "Climber/TargetRotations",
          Math.abs(
              pivot.getPosition().getValueAsDouble()
                  - (Math.signum(pigeon.getRoll().getValueAsDouble())
                      * (((180.0 - Math.abs(pigeon.getRoll().getValueAsDouble())) / 360.0)
                          * 592))));
      SmartDashboard.putNumber("Climber/PigeonRoll", pigeon.getRoll().getValueAsDouble());
    }
    Logger.recordOutput("Fast", 1.0 / (currentTime - lastTime));
    lastTime = currentTime;
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
            .until(() -> lift.getStatorCurrent().getValueAsDouble() > 25.0),
        new InstantCommand(() -> lift.setPosition(0.0), this));
    //    .andThen(new RunCommand(() -> lift.setControl(new DutyCycleOut(0.0)), this));
  }

  public Command goToGrip() {
    return new RunCommand(
        () -> {
          pivotGotoPosition(pivotGripTarget.getValue());
          liftGotoPosition(liftGripTarget.getValue());
        },
        this);
  }

  public Command goToChinStrap() {
    return new RunCommand(() -> liftGotoPosition(liftChinStrapTarget.getValue()), this);
  }

  public Command getOnPole() {
    return new RunCommand(() -> liftGetOnPole(), this);
  }

  public Command gotoL1() {
    return new RunCommand(() -> pivotGotoPosition(pivotL1Target.getValue()), this)
        .withTimeout(0.5)
        .andThen(
            (() -> {
              liftGotoPosition(liftTuckTarget.getValue());
              pivotGotoPosition(pivotL1Target.getValue());
            }),
            this);
  }

  public Command gotoStow() {
    return new RunCommand(
        () -> {
          liftGotoPosition(liftDownTarget.getValue());
          pivotGotoPosition(pivotStowTarget.getValue());
        },
        this);
  }

  public Command flipCommand() { // assume L1
    return new RunCommand(() -> flip(), this);
  }

  public Command stopall() {
    return new RunCommand((() -> stop()), this);
  }

  public Command pivotGotToL1() {
    return new RunCommand(() -> pivotGotoPosition(pivotL1Target.getValue()), this);
  }
}
