package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class temp {
    public Translation2d robotToTurret = new Translation2d(0.3, -3/4 * Math.PI); //m
    public Translation2d goalPosition = new Translation2d(2, 2);
    public static Rotation2d quarterRotation = Rotation2d.fromRadians(Math.PI/2);

    //TO GO IN CONSTANTS ^^

    public Translation2d robotPosition = new Translation2d(1,1); //m
    public Rotation2d robotYaw = new Rotation2d(Math.PI * (1/3)); //rad
    public Translation2d robotVelocity = new Translation2d(); //m/s
    public double robotRotationalVelocity = 0.1; //rad/s
    
    public double timeOfFlight = 0; //seconds
    public Translation2d turretToTarget; //m; distanceToTarget
    public int numberOfIterations = 5;

    public void periodic() {
        if (robotVelocity.getNorm() > (1/25)) {
            newton();
        }
        else {
            timeOfFlight = 0;
        }
    }

    public void newton() {
        Translation2d turretDistanceToTarget = goalPosition
            .minus(robotPosition).minus(robotToTurret.rotateBy(robotYaw));
        Translation2d turretRelativeVelocityToTarget = robotVelocity
            .plus(robotToTurret.rotateBy(robotYaw.plus(quarterRotation)).times(robotRotationalVelocity));
        if (timeOfFlight==0) {
            timeOfFlight = lookup(turretDistanceToTarget.getNorm());
        }
        int i;
        for (i=0; i<numberOfIterations; i++) {
            Translation2d projectedDistance = turretDistanceToTarget.minus(turretRelativeVelocityToTarget.times(timeOfFlight));
            timeOfFlight = timeOfFlight -
            (timeOfFlight - lookup(projectedDistance.getNorm()))
            /(1 - (lookupPrime(projectedDistance.getNorm()))*
            (projectedDistance.dot(turretRelativeVelocityToTarget.unaryMinus())/projectedDistance.getNorm()));
            }
    }

    public double lookup(double distance) {
        return Math.sqrt(distance);
    }

    public double lookupPrime(double distance) {
        return 1/(2 * Math.sqrt(distance));
    }

    }
