package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Retractomatic extends SubsystemBase {
  private TalonFX retractomatic = new TalonFX(Constants.RETRACTOMATIC, CANBus.roboRIO());

  private TorqueCurrentFOC request = new TorqueCurrentFOC(0.0);
  
  private DoubleSupplier m_facingSup;
  private DoubleSupplier m_velSup;

  public Retractomatic(DoubleSupplier facingSup, DoubleSupplier velSup) {
    m_facingSup = facingSup;
    m_velSup = velSup;
  }

  private void setDefault() {
    double currentFacingAngle = m_facingSup.getAsDouble();
    double currentVelocity = m_velSup.getAsDouble();
    double targetCurrent = 0.0;

    //First Quadrant, negative (clockwise) velocity, pull in
    if((currentFacingAngle > -90.0 && currentFacingAngle < 0.0) && currentVelocity < 0.0) {

    } else if(currentFacingAngle < -90.0) {

    }
  }

  public Command setDefualtCommand() {
    return new RunCommand(() -> setDefault(), this);
  }
}
