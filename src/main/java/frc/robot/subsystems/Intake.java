package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
  private final TalonFX intakeRoller =
      new TalonFX(Constants.INTAKE_ROLLER, new CANBus("Drivetrain"));
  // private final TalonFX intakePivot = new TalonFX(Constants.INTAKE_PIVOT, CANBus.roboRIO());

  private VelocityVoltage request = new VelocityVoltage(0.0);
  private MotionMagicVoltage pivotRequest = new MotionMagicVoltage(0.0);

  private DoublePreferenceConstant intakeSpeed = new DoublePreferenceConstant("Intake/Speed", 0.8);
  public double commandedVelocity = 0;

  private MotionMagicPIDPreferenceConstants intakeConfigConstants =
      new MotionMagicPIDPreferenceConstants("IntakeMotors");

  public Intake() {
    configureTalons();
  }

  public void periodic() {
    SmartDashboard.putNumber(
        "Intake/RollerVelocity", intakeRoller.getVelocity().getValueAsDouble());
  }

  private void configureTalons() {
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.Slot0.kP = intakeConfigConstants.getKP().getValue();
    intakeConfig.Slot0.kI = intakeConfigConstants.getKI().getValue();
    intakeConfig.Slot0.kD = intakeConfigConstants.getKD().getValue();
    intakeConfig.Slot0.kV = intakeConfigConstants.getKV().getValue(); // 0.125
    intakeConfig.Slot0.kS = intakeConfigConstants.getKS().getValue();
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    intakeRoller.getConfigurator().apply(intakeConfig);
  }

  private void setSpeed(DoubleSupplier speed) {
    intakeRoller.setControl(
        request.withVelocity(
            speed.getAsDouble())); // * (commandedVelocity / 4.58))); // .withUpdateFreqHz(1000.0));
  }

  private void stopMotors() {
    intakeRoller.stopMotor();
  }

  public void setCommandedVelocity(double velocity) {
    commandedVelocity = velocity;
  }

  public Command runIntake() {
    return new RunCommand(() -> setSpeed(() -> intakeSpeed.getValue()), this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopMotors(), this);
  }
}
