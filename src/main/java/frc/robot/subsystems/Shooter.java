package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
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
    private boolean boosted = false;
    //private Trigger boostStarted = new Trigger(() -> boosted);
    private Timer timeSinceBallLastSeen = new Timer();
    //private Timer timeSinceBoostStarted = new Timer();

    private int ballsCount;
    private double earliestBallTime = 0;
    private double lastBallTime;
    private double BallsPerSecond;

    private VelocityDutyCycle requestShooter = new VelocityDutyCycle(0.0);
    private VoltageOut voltagerequest = new VoltageOut(0);
    public DoublePreferenceConstant shootSpeed = new DoublePreferenceConstant("Shooter/ShootSpeed", 0.0);
    public DoublePreferenceConstant shootVoltage = new DoublePreferenceConstant("Shooter/ShootVoltage", 0.0);
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
        feederBeamBreakTrigger.onTrue(new InstantCommand(() -> calculateBPS()));
        //boostStarted.onTrue(new InstantCommand(() -> {timeSinceBoostStarted.reset(); timeSinceBoostStarted.start();}));
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterVoltage", shooterMain.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/ShooterCurrent", shooterMain.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Shooter/TimeSinceBallLastSeen", timeSinceBallLastSeen.get());
        SmartDashboard.putNumber("Shooter/BallsPerSecond", (Math.round((100.0 * BallsPerSecond)) / 100.0)); //round to hundredths
        //SmartDashboard.putNumber("Shooter/TimeSinceBoostStarted", timeSinceBoostStarted.get());
        SmartDashboard.putBoolean("Shooter/IsBeamBlocked", isBeamBlocked());
        SmartDashboard.putBoolean("Shooter/Boosted", boosted);
    }

    private void setShooterSpeed(DoubleSupplier speed, DoubleSupplier FeedForwardIncrease) {
        if (shooterMain.getVelocity().getValueAsDouble() >= (speed.getAsDouble())) { // normal
            shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()).withFeedForward(0.0));
            boosted = false;
        }
        else if ((timeSinceBallLastSeen.get() >= (increaseDuration.getValue() + increaseDelay.getValue())) || (timeSinceBallLastSeen.get() <= increaseDelay.getValue())) {
            shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()).withFeedForward(0.0));  // normal or delay time
            boosted = false;
        }
        else { // in boost duration
            //double boost = FeedForwardIncrease.getAsDouble() * 
             //   ((increaseDuration.getValue() + increaseDelay.getValue() - timeSinceBallLastSeen.get())/(2 * increaseDuration.getValue()));
            double boost = FeedForwardIncrease.getAsDouble();
            shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()).withFeedForward(boost));
            boosted = true;
        } //this runs if ((timeSinceBallLastSeen.get() > increaseDelay.getValue()) && (timeSinceBallLastSeen.get() < (increaseDuration.getValue() + increaseDelay.getValue()))
    }

    private void setShooterVoltage(DoubleSupplier voltage) {
        shooterMain.setControl(voltagerequest.withOutput(voltage.getAsDouble()));
    }

    private void stopShooterMotors() {
        shooterMain.stopMotor();
        // shooterFollower.stopMotor();
    }

    private void calculateBPS() {
        ballsCount = ballsCount + 1;
        double currentTime = Timer.getFPGATimestamp();
        if (earliestBallTime==0) { //if it is not yet set
            earliestBallTime = currentTime;
        } 
        else {
            lastBallTime = currentTime;
        }
        if (earliestBallTime > 0 && lastBallTime > 0) {
            BallsPerSecond = (ballsCount)/(lastBallTime-earliestBallTime);
        }
    }

    public void resetBPS() {
        ballsCount = 0;
        earliestBallTime = 0;
        lastBallTime = 0;
    }

    private boolean isBeamBlocked() {
        return !feederBeamBreak.get();
    } 

    public Command runShooter() {
        return new RunCommand(() -> setShooterSpeed(() -> shootSpeed.getValue(), () -> increaseFeedForward.getValue()), this);
    }

    public Command runShooterVoltage() {
        return new RunCommand(() -> setShooterVoltage(() -> shootVoltage.getValue()));
    }

    public Command stopShooter() {
        return new RunCommand(() -> stopShooterMotors(), this);
    }
}