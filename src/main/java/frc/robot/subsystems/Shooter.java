package frc.robot.subsystems;

import java.io.FileFilter;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMain = new TalonFX(12, CANBus.roboRIO()); //forward +
    private TalonFX shooterFollower = new TalonFX(3, CANBus.roboRIO()); //forward -
    private DigitalInput feederBeamBreak = new DigitalInput(0);
    private Trigger feederBeamBreakTrigger = new Trigger(() -> isBeamBlocked());
    private Timer timeSinceBallLastSeen = new Timer();

    private VelocityDutyCycle requestShooter = new VelocityDutyCycle(0.0);
    public DoublePreferenceConstant shootSpeed = new DoublePreferenceConstant("Shooter/ShootSpeed", 0.0);
    public DoublePreferenceConstant increaseDuration = new DoublePreferenceConstant("Shooter/IncreaseDuration", 0.0);
    public DoublePreferenceConstant increaseDelay = new DoublePreferenceConstant("Shooter/IncreaseDelay", 0.0);
    public DoublePreferenceConstant increaseFeedForward = new DoublePreferenceConstant("Shooter/IncreaseFeedForward", 0.0);

    private MotionMagicPIDPreferenceConstants shooterConfigConstants = new MotionMagicPIDPreferenceConstants("ShooterMotors");


    public Shooter() {
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();

        shooterConfig.Slot0.kP = shooterConfigConstants.getKP().getValue();
        shooterConfig.Slot0.kI = shooterConfigConstants.getKI().getValue();
        shooterConfig.Slot0.kD = shooterConfigConstants.getKD().getValue();
        shooterConfig.Slot0.kV = shooterConfigConstants.getKV().getValue();
        shooterConfig.Slot0.kS = shooterConfigConstants.getKS().getValue();
        shooterMain.getConfigurator().apply(shooterConfig);
        // shooterFollower.getConfigurator().apply(shooterConfig);
        shooterFollower.setControl(new Follower(12, MotorAlignmentValue.Opposed));
        timeSinceBallLastSeen.reset();
        feederBeamBreakTrigger.onTrue(new InstantCommand(() -> {timeSinceBallLastSeen.reset(); timeSinceBallLastSeen.start();}));
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterVoltage", shooterMain.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TimeSinceBallLastSeen", timeSinceBallLastSeen.get());
        SmartDashboard.putBoolean("Shooter/IsBeamBlocked", isBeamBlocked());
    }

    private void setShooterSpeed(DoubleSupplier speed, DoubleSupplier FeedForwardIncrease) {
        if ((timeSinceBallLastSeen.get() >= (increaseDuration.getValue() + increaseDelay.getValue())) || (timeSinceBallLastSeen.get() <= increaseDelay.getValue())) {
            shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()));  // normal or delay time
        }
        else { // in boost duration
            shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()).withFeedForward(FeedForwardIncrease.getAsDouble()));
        } //this runs if ((timeSinceBallLastSeen.get() > increaseDelay.getValue()) && (timeSinceBallLastSeen.get() < (increaseDuration.getValue() + increaseDelay.getValue()))
    }

    private void stopShooterMotors() {
        shooterMain.stopMotor();
        // shooterFollower.stopMotor();
    }

    private boolean isBeamBlocked() {
        return !feederBeamBreak.get();
    } 

    public Command runShooter() {
        return new RunCommand (() -> setShooterSpeed(() -> shootSpeed.getValue(), () -> increaseFeedForward.getValue()), this);
    }

    public Command stopShooter() {
        return new RunCommand(() -> stopShooterMotors(), this);
    }
}