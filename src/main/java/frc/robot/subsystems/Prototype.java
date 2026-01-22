package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Prototype extends SubsystemBase {
    
    private final TalonFX indexer1 = new TalonFX(19, CANBus.roboRIO());
    private final TalonFX indexer2 = new TalonFX(18, CANBus.roboRIO());
    private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
    private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

    private DoublePreferenceConstant speed = new DoublePreferenceConstant("Hopper/Speed", 0.8);

    public Prototype() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Slot0.kP = 
        // config.Slot0.kI = 
        // config.Slot0.kD = 
        // config. = 
    }

    private void setSpeed(double speed) {
        indexer1.setControl(requestcycle.withOutput(-speed));
        indexer2.setControl(requestcycle.withOutput(-speed));
    }

    private void stopMotors() {
        indexer1.stopMotor();
        indexer2.stopMotor();
    }

    public Command runIndexer() {
        return new RunCommand(() -> setSpeed(speed.getValue()), this);
    }

    public Command stop() {
        return new RunCommand(() -> stopMotors(), this);
    }
}


