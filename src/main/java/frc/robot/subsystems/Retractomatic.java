package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

public class Retractomatic extends SubsystemBase {
  private TalonFX retractomatic = new TalonFX(Constants.RETRACTOMATIC, CANBus.roboRIO());

  private double goingOutCurrent = 0.0;
  private double goingInCurrent = -40.0;
  private TorqueCurrentFOC request = new TorqueCurrentFOC(0.0);

  private DoubleSupplier m_facingSup;
  private DoubleSupplier m_velSup;

  public Retractomatic(DoubleSupplier facingSup, DoubleSupplier velSup) {
    m_facingSup = facingSup;
    m_velSup = velSup;

    configureMotors();
  }

  private void configureMotors() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    retractomatic.getConfigurator().apply(config);
  }

  private void setDefault() {
    double currentFacingAngleRelative =
        m_facingSup.getAsDouble() + 10.0; // TODO: find facing that is minimum tether length
    double currentVelocity = m_velSup.getAsDouble();
    double targetCurrent = 0.0; // POSITIVE OUT

    // CCW side of minimum tether length, negative (clockwise) velocity, pull in
    if (currentFacingAngleRelative > 0.0 && currentVelocity < 0.0) { // CCW side of 0; going in
      targetCurrent = goingInCurrent;
    } else if (currentFacingAngleRelative > 0.0
        && currentVelocity > 0.0) { // CCW side of 0; going out
      targetCurrent = goingOutCurrent;
    } else if (currentFacingAngleRelative < 0.0
        && currentVelocity < 0.0) { // CW side of 0; going out
      targetCurrent = goingOutCurrent;
    } else if (currentFacingAngleRelative < 0.0
        && currentVelocity > 0.0) { // CW side of 0; going in
      targetCurrent = goingInCurrent;
    } else if (currentFacingAngleRelative == 0.0
        || currentVelocity == 0.0) { // At min length or not moving
      targetCurrent = 0.0;
    } else { // should never find this state
      targetCurrent = 0.0;
      System.out.println(
          "Strange Retractomatic State" + currentFacingAngleRelative + currentVelocity);
    }

    retractomatic.setControl(request.withOutput(targetCurrent));
  }

  public void stop() {
    retractomatic.stopMotor();
  }

  public Command setDefualtCommand() {
    return new RunCommand(() -> setDefault(), this);
  }

  public Command setStop() {
    return new RunCommand(() -> stop(), this);
  }
}
