package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private final CANdle m_candle = new CANdle(0);

  public Lights() {
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.On;
    configAll.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
    configAll.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    configAll.LED.StripType = StripTypeValue.RGB;
    configAll.LED.BrightnessScalar = 1.0;
    configAll.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
    m_candle.getConfigurator().apply(configAll);
  }
}
