package frc.robot.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preflib.PrefGroup;
import java.util.HashMap;
import java.util.Map;

public abstract class BaseRobot extends SubsystemBase {

  private Map<String, WrapperMotor> allMotors = new HashMap<>();
  private Map<String, PrefGroup> preferences = new HashMap<>();

  enum Configs {
    PID,
    MOTION_MAGIC,
    MOTOR_OUTPUTS,
  }

  private class WrapperMotor {
    TalonFX motor;
    Configs[] configs;

    public WrapperMotor(String name, int id, CANBus bus, Configs... conf) {
      motor = new TalonFX(id, bus);
      configs = conf;
    }

    public TalonFX getMotor() {
      return motor;
    }
  }

  public BaseRobot() {
    super();
    initMotors();
  }

  private void initMotors() {}

  private TalonFX createTalon(String name, int id, CANBus bus, Configs... conf) {
    WrapperMotor motor = new WrapperMotor(name, id, bus, conf);
    allMotors.put(name, motor);
    return motor.getMotor();
  }
}
