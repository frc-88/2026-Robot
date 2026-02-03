package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    
    LinearFilter f = LinearFilter.singlePoleIIR(0.1, 0.02);
    CANrange caNrange = new CANrange(8);
    Debouncer debouncer = new Debouncer(0.15);
    private boolean below = false;
    public Climber() {
        CANrangeConfiguration congifCaNrangeConfiguration = new CANrangeConfiguration(); //congif
        congifCaNrangeConfiguration.ProximityParams.MinSignalStrengthForValidMeasurement = 50000;
        congifCaNrangeConfiguration.FovParams.FOVRangeX = 6.75;
        congifCaNrangeConfiguration.FovParams.FOVRangeY = 6.75;
        congifCaNrangeConfiguration.ProximityParams.ProximityThreshold = 0.1;
        congifCaNrangeConfiguration.ProximityParams.ProximityHysteresis = 0.02;

        caNrange.getConfigurator().apply(congifCaNrangeConfiguration);
    }

    public boolean isDetected() {
        double distance = f.calculate(caNrange.getDistance().getValueAsDouble());
        if (debouncer.calculate(caNrange.getSignalStrength().getValueAsDouble() > 50000) == false) {
            below = false;
            return false; //unknown state
        }
        else if (distance > 0.12) {
            below = false;
            return false; //too far
        }
        else if (distance < 0.08) {
            below = true;
            return true;

        }
        else if (distance < 0.12 && distance > 0.08) {
            if (below) {
                return true; // it was below .08 before
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    public void periodic() {
        SmartDashboard.putNumber("caNrange", f.calculate(caNrange.getDistance().getValueAsDouble()));
        SmartDashboard.putBoolean("caNrangeDetected", isDetected());

    }

}
