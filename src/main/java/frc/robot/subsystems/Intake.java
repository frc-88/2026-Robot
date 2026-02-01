package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final TalonFX intake = new TalonFX(Constants.INTAKE_MAIN, CANBus.roboRIO());
  private final var supplyCurrent = intake.getSupplyCurrent();
  private final var motorVoltage = intake.getMotorVoltage();
  private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
  private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

  private DoublePreferenceConstant speed = new DoublePreferenceConstant("Intake/Speed", 0.8);

  public Intake() {
    configureTalons();
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(supplyCurrent, motorVoltage);
    double supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    double voltage = motorVoltage.getValueAsDouble();
    double powerWatts = voltage * supplyCurrentAmps;

    Logger.recordOutput("Intake/SupplyCurrentAmps", supplyCurrentAmps);
    Logger.recordOutput("Intake/PowerWatts", powerWatts);
    Logger.recordOutput("Intake/MotorVoltage", voltage);

    SmartDashboard.putNumber("Intake/SupplyCurrentAmps", supplyCurrentAmps);
    SmartDashboard.putNumber("Intake/PowerWatts", powerWatts);
    SmartDashboard.putNumber("Intake/MotorVoltage", voltage);
  }

  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // config.Slot0.kP =
    // config.Slot0.kI =
    // config.Slot0.kD =
    // config. =
    config.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // this might not work yet
    intake.getConfigurator().apply(config);
  }

  private void setSpeed(double speed) {
    intake.setControl(requestcycle.withOutput(speed));
  }

  private void stopMotors() {
    intake.stopMotor();
  }

  public Command runIndexer() {
    return new RunCommand(() -> setSpeed(speed.getValue()), this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopMotors(), this);
  }
}
