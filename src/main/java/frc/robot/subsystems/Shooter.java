package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.preferenceconstants.DoublePreferenceConstant;
import frc.robot.util.preferenceconstants.MotionMagicPIDPreferenceConstants;

public class Shooter extends SubsystemBase {
    private TalonFX shooterMain = new TalonFX(12, CANBus.roboRIO()); //forward +
    private TalonFX shooterFollower = new TalonFX(2, CANBus.roboRIO()); //forward -

    private VelocityDutyCycle requestShooter = new VelocityDutyCycle(0.0);
    private DoublePreferenceConstant shootSpeed = new DoublePreferenceConstant("Turret/ShootSpeed", 0.0);
    private MotionMagicPIDPreferenceConstants shooterConfigConstants = new MotionMagicPIDPreferenceConstants("TurretMainMotor");

    private final VoltageOut m_voltReq = new VoltageOut(0.0);
    private final SysIdRoutine m_sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,        // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null,        // Use default timeout (10 s)
                            // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                this::setVoltage,
                null,
                this
            )
        );

    // Volts per (radian per second)
    private static final double kFlywheelKv = 0.023; // TODO: determine with SysId

    // Volts per (radian per second squared)
    private static final double kFlywheelKa = 0.001; // TODO: determine with SysId

    // The plant holds a state-space model of our flywheel. This system has the following properties:
    //
    // States: [velocity], in radians per second.
    // Inputs (what we can "put in"): [voltage], in volts.
    // Outputs (what we can measure): [velocity], in radians per second.
    //
    // The Kv and Ka constants are found using the FRC Characterization toolsuite.
    private final LinearSystem<N1, N1, N1> m_flywheelPlant =
        LinearSystemId.identifyVelocitySystem(kFlywheelKv, kFlywheelKa);        

    // The observer fuses our encoder data and voltage inputs to reject noise.
    private final KalmanFilter<N1, N1, N1> m_observer =
        new KalmanFilter<>(
            Nat.N1(),
            Nat.N1(),
            m_flywheelPlant,
            VecBuilder.fill(3.0), // How accurate we think our model is TODO: Tune
            VecBuilder.fill(0.01), // How accurate we think our encoder TODO: Tune
            // data is
            0.020);

      // A LQR uses feedback to create voltage commands.
    private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
        new LinearQuadraticRegulator<>(
            m_flywheelPlant,
            VecBuilder.fill(8.0), // TODO: Tune
            // qelms. Velocity error tolerance, in radians per second. Decrease
            // this to more heavily penalize state excursion, or make the controller behave more
            // aggressively.
            VecBuilder.fill(12.0), // TODO: Tune
            // relms. Control effort (voltage) tolerance. Decrease this to more
            // heavily penalize control effort, or make the controller less aggressive. 12 is a good
            // starting point because that is the (approximate) maximum voltage of a battery.
            0.020); 
            // Nominal time between loops. 0.020 for TimedRobot, but can be
            // lower if using notifiers.

    // The state-space loop combines a controller, observer, feedforward and plant for easy control.
    private final LinearSystemLoop<N1, N1, N1> m_loop =
        new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

    public Shooter() {
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
    }

    public void periodic() {
        SmartDashboard.putNumber("Shooter/ShooterVelocity", shooterMain.getVelocity().getValueAsDouble());
    }

    private void setShooterSpeed(DoubleSupplier speed) {
        shooterMain.setControl(requestShooter.withVelocity(speed.getAsDouble()));
    }

    private void setVoltage(Voltage volts) {
        shooterMain.setControl(m_voltReq.withOutput(volts));
    }

    private void setStateSpaceController(double speed) {
        // Sets the target speed of our flywheel. This is similar to setting the setpoint of a
        // PID controller.
        m_loop.setNextR(VecBuilder.fill(speed));
    }

    private void runStateSpaceController() {
        // Correct our Kalman filter's state vector estimate with encoder data.
        m_loop.correct(VecBuilder.fill(shooterMain.getVelocity().getValueAsDouble()));

        // Update our LQR to generate new voltage commands and use the voltages to predict the next
        // state with out Kalman filter.
        m_loop.predict(0.020);

        // Send the new calculated voltage to the motors.
        // voltage = duty cycle * battery voltage, so
        // duty cycle = voltage / battery voltage
        double nextVoltage = m_loop.getU(0);
        this.setVoltage(Volts.of(nextVoltage));
    }

    private void stopShooterMotors() {
        shooterMain.stopMotor();
        shooterFollower.stopMotor();
    }

    public Command runFlywheel() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setStateSpaceController(shootSpeed.getValue()), this),
            new RunCommand (() -> runStateSpaceController(), this)
        );
    }

    public Command stopFlywheel() {
        return new SequentialCommandGroup(
            new InstantCommand(() -> setStateSpaceController(0), this),
            new RunCommand (() -> runStateSpaceController(), this)
        );
    }

    public Command runShooter() {
        return new RunCommand (() -> setShooterSpeed(() -> shootSpeed.getValue()), this);
    }

    public Command stopShooter() {
        return new RunCommand(() -> stopShooterMotors(), this);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

}