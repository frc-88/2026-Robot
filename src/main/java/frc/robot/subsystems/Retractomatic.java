package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Retractomatic extends SubsystemBase {
  private TalonFX retractomatic = new TalonFX(Constants.RETRACTOMATIC, CANBus.roboRIO());
}
