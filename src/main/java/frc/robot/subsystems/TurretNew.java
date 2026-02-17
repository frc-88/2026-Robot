package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class TurretNew {
    private TalonFX turret = new TalonFX(Constants.TURRET_MOTOR_ID, CANBus.roboRIO());
    private CANcoder cancoderCaNcoder = new CANcoder(Constants.CANCODER_, CANBus.roboRIO());
    private CANcoder cancoderCaNcoder1 = new CANcoder(Constants.CANCODER_, CANBus.roboRIO());
}
