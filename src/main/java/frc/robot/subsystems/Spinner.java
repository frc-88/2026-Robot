package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Spinner extends SubsystemBase {
private final TalonFX spinner = new TalonFX(6, CANBus.roboRIO());
    private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
    //private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

    private DoublePreferenceConstant spinnerSpeed = new DoublePreferenceConstant("Spinner/SpinnerSpeed", 0.0);
    
    private MotionMagicPIDPreferenceConstants spinnerConfigConstants = new MotionMagicPIDPreferenceConstants("SpinnerMotors");


    public Spinner() {
        configureTalons();
    }

    public void periodic() {
        SmartDashboard.putNumber("Spinner/SpinnerVelocity", spinner.getVelocity().getValueAsDouble());
    }

    private void configureTalons() {
        TalonFXConfiguration spinnerConfig = new TalonFXConfiguration();
        spinnerConfig.Slot0.kP = spinnerConfigConstants.getKP().getValue();
        spinnerConfig.Slot0.kI = spinnerConfigConstants.getKI().getValue();
        spinnerConfig.Slot0.kD = spinnerConfigConstants.getKD().getValue();
        spinnerConfig.Slot0.kV = spinnerConfigConstants.getKV().getValue();
        spinnerConfig.Slot0.kS = spinnerConfigConstants.getKS().getValue();
        spinnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        spinner.getConfigurator().apply(spinnerConfig);
    }

    private void setSpinnerSpeed(DoubleSupplier speed) {
        spinner.setControl(request.withVelocity(speed.getAsDouble()));
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
