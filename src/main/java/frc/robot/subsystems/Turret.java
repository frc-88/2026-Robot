package frc.robot.subsystems;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Turret extends SubsystemBase {
    
    private TalonFX shooterMain = new TalonFX(3, CANBus.roboRIO()); //forward +
    private TalonFX shooterFollower = new TalonFX(2, CANBus.roboRIO()); //forward -

    private VelocityDutyCycle request = new VelocityDutyCycle(0.0);
    private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

    private DoublePreferenceConstant speed = new DoublePreferenceConstant("Turret/Speed", 0.0);
    private MotionMagicPIDPreferenceConstants mainConfig = new MotionMagicPIDPreferenceConstants("TurretMainMotor");

    public Turret() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = mainConfig.getKP().getValue();
        config.Slot0.kI = mainConfig.getKI().getValue();
        config.Slot0.kD = mainConfig.getKD().getValue();
        shooterMain.getConfigurator().apply(config);
        shooterFollower.getConfigurator().apply(config);
        shooterFollower.setControl(new Follower(3, MotorAlignmentValue.Opposed));
    
    }

    private void setSpeed(double speed) {
        shooterMain.setControl(request.withVelocity(speed));
    }

    private void stopMotors() {
        shooterMain.stopMotor();
        shooterFollower.stopMotor();
    }

    public Command runTurret() {
        return new RunCommand(() -> setSpeed(speed.getValue()), this);
    }

    public Command stopTurret() {
        return new RunCommand(() -> stopMotors(), this);
    }
}
