package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class Feeder extends SubsystemBase {

  private final TalonFX feeder = new TalonFX(Constants.FEEDER_MAIN, CANBus.roboRIO());

  private final VelocityVoltage request = new VelocityVoltage(0.0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final DoublePreferenceConstant feedSpeed =
      new DoublePreferenceConstant("Feeder/FeedSpeed", 0.0);
  private final MotionMagicPIDPreferenceConstants feederConfigConstants =
      new MotionMagicPIDPreferenceConstants("Feeder/MotorPID");

  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // Use default ramp rate (1 V/s)
              Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
              null, // Use default timeout (10 s)
              // Log state with Phoenix SignalLogger class
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(this::setVoltage, null, this));

  public Feeder() {
    configureTalons();

    SmartDashboard.putData("Feeder/RunFeeder", runFeeder());
    SmartDashboard.putData("Feeder/StopFeeder", stopFeeder());
    SmartDashboard.putData(
        "Feeder/SysId/Quasistatic Forward", sysIdQuasistatic(Direction.kForward));
    SmartDashboard.putData(
        "Feeder/SysId/Quasistatic Reverse", sysIdQuasistatic(Direction.kReverse));
    SmartDashboard.putData("Feeder/SysId/Dynamic Forward", sysIdDynamic(Direction.kForward));
    SmartDashboard.putData("Feeder/SysId/Dynamic Reverse", sysIdDynamic(Direction.kReverse));
  }

  private void configureTalons() {
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    feederConfig.Slot0.kP = feederConfigConstants.getKP().getValue();
    feederConfig.Slot0.kI = feederConfigConstants.getKI().getValue();
    feederConfig.Slot0.kD = feederConfigConstants.getKD().getValue();
    feederConfig.Slot0.kV = feederConfigConstants.getKV().getValue();
    feederConfig.Slot0.kS = feederConfigConstants.getKS().getValue();
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    feeder.getConfigurator().apply(feederConfig);

    feeder.getVelocity().setUpdateFrequency(100);
  }

  @AutoLogOutput(key = "Feeder/Voltage")
  private Voltage getVoltage() {
    return feeder.getMotorVoltage().getValue();
  }

  @AutoLogOutput(key = "Feeder/Current")
  private Current getCurrent() {
    return feeder.getStatorCurrent().getValue();
  }

  @AutoLogOutput(key = "Feeder/Velocity")
  private AngularVelocity getVelocity() {
    return feeder.getVelocity().getValue();
  }

  private void setVoltage(Voltage volts) {
    feeder.setControl(m_voltReq.withOutput(volts));
  }

  private void setFeederSpeed(DoubleSupplier speed) {
    feeder.setControl(request.withVelocity(speed.getAsDouble()));
  }

  private void stopFeederMotors() {
    feeder.stopMotor();
  }

  public void periodic() {}

  public Command runFeeder() {
    return new RunCommand(() -> setFeederSpeed(() -> feedSpeed.getValue()), this);
  }

  public Command stopFeeder() {
    return new RunCommand(() -> stopFeederMotors(), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}
