package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Intake extends SubsystemBase {

  private final TalonFX intakePivot = new TalonFX(Constants.INTAKE_MAIN, CANBus.roboRIO());
  private PWM intakeSpinner = new PWM(0);
  private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

  private DoublePreferenceConstant speed = new DoublePreferenceConstant("Intake/Speed", 0.8);

  public Intake() {
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    // config.Slot0.kP =
    // config.Slot0.kI =
    // config.Slot0.kD =
    // config. =
    config.MotorOutput.Inverted =
        InvertedValue.CounterClockwise_Positive; // this might not work yet
    intakePivot.getConfigurator().apply(config);
  }

  // private void setSpeed(double speed) {
  //   intake.setControl(requestcycle.withOutput(speed));
  // }

  // private void stopMotors() {
  //   intake.stopMotor();
  // }

  private void setSpeed(double speed) {
    intakeSpinner.setSpeed(speed);
  }

  private void stopMotors() {
    intakeSpinner.setSpeed(0.0);
  }

  public Command runIntake() {
    return new RunCommand(() -> setSpeed(speed.getValue()), this);
  }

  public Command stopIntake() {
    return new RunCommand(() -> stopMotors(), this);
  }
}
