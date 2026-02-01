package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;

public class Spinner extends SubsystemBase {
private final TalonFX spinner = new TalonFX(6, CANBus.roboRIO());
    private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
    private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

    private DoublePreferenceConstant spinnerSpeed = new DoublePreferenceConstant("Spinner/SpinnerSpeed", 0.0);

    public Spinner() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        // config.Slot0.kP = 
        // config.Slot0.kI = 
        // config.Slot0.kD = 
        // config. = 
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spinner.getConfigurator().apply(config);
    }

    private void setSpinnerSpeed(DoubleSupplier speed) {
        spinner.setControl(requestcycle.withOutput(speed.getAsDouble()));
    }

    private void stopSpinnerMotors() {
        spinner.stopMotor();
    }

    public Command runSpinner() {
        return new RunCommand(() -> setSpinnerSpeed(() -> spinnerSpeed.getValue()), this);
    }

    public Command stopSpinner() {
        return new RunCommand(() -> stopSpinnerMotors(), this);
    }
}
