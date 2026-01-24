package frc.robot.subsystems;

import java.util.function.DoublePredicate;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Turret extends SubsystemBase {
    
    private TalonFX shooterMain = new TalonFX(12, CANBus.roboRIO()); //forward +
    private TalonFX shooterFollower = new TalonFX(2, CANBus.roboRIO()); //forward -
    private TalonFX shooterFeeder = new TalonFX(18, CANBus.roboRIO());

    private VelocityDutyCycle requestShooter = new VelocityDutyCycle(0.0);
    private VelocityDutyCycle requestFeeder = new VelocityDutyCycle(0.0);
    //private DutyCycleOut requestcycle = new DutyCycleOut(0.0);

    private DoublePreferenceConstant shootSpeed = new DoublePreferenceConstant("Turret/ShootSpeed", 0.0);
    private DoublePreferenceConstant feedSpeed = new DoublePreferenceConstant("Turret/FeedSpeed", 0.0);
    private MotionMagicPIDPreferenceConstants shooterConfigConstants = new MotionMagicPIDPreferenceConstants("TurretMainMotor");
    private MotionMagicPIDPreferenceConstants feederConfigConstants = new MotionMagicPIDPreferenceConstants("FeederMotor");

    public Turret() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
        shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
        shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
        shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
        shooterMain.getConfigurator().apply(shooterConfig);
        shooterFollower.getConfigurator().apply(shooterConfig);
        shooterFollower.setControl(new Follower(3, MotorAlignmentValue.Opposed));


        TalonFXConfiguration feederConfig = new TalonFXConfiguration();

        feederConfig.Slot0.kP = feederConfigConstants.getKP().getValue();
        feederConfig.Slot0.kI = feederConfigConstants.getKI().getValue();
        feederConfig.Slot0.kD = feederConfigConstants.getKD().getValue();
        feederConfig.Slot0.kV = feederConfigConstants.getKV().getValue();
        shooterFeeder.getConfigurator().apply(feederConfig);
    }

    public void periodic() {
        SmartDashboard.putNumber("Turret/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Turret/FeederVelocity", shooterFeeder.getVelocity().getValueAsDouble());
    }

    private void setShooterSpeed(double speed) {
        shooterMain.setControl(requestShooter.withVelocity(speed));
    }

    private void setFeederSpeed(double speed) {
        shooterFeeder.setControl(requestFeeder.withVelocity(speed));
    }

    private void setBothSpeeds() {
        setShooterSpeed(shootSpeed.getValue());
        setFeederSpeed(feedSpeed.getValue());
    }

    private void stopAllMotors() {
        shooterMain.stopMotor();
        shooterFollower.stopMotor();
        shooterFeeder.stopMotor();
    }

    private void stopOnlyShooterMotors() {
        shooterMain.stopMotor();
        shooterFollower.stopMotor();
    }

    private void stopOnlyFeederMotors() {
        shooterFeeder.stopMotor();
    }

    public Command runAllTurret() {
        return new RunCommand(() -> setBothSpeeds(), this);
    }

    public Command runOnlyFeeder() {
        return new RunCommand (() -> setFeederSpeed(feedSpeed.getValue()), this);
    }

    public Command runOnlyShooter() {
        return new RunCommand (() -> setShooterSpeed(shootSpeed.getValue()), this);
    }

    public Command stopAllTurret() {
        return new RunCommand(() -> stopAllMotors(), this);
    }
     
    public Command stopOnlyFeeder() {
        return new RunCommand(() -> stopOnlyFeederMotors(), this);
    }
    
    public Command stopOnlyShooter() {
        return new RunCommand(() -> stopOnlyShooterMotors(), this);
    }
}

