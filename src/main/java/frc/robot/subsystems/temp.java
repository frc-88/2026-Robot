package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class temp {
    public Translation2d robotPosition = new Translation2d(1,1); //m
    public Translation2d robotToTurret = new Translation2d(0.3, -3/4 * Math.PI); //m
    public Rotation2d robotYaw = new Rotation2d(Math.PI * (1/3)); //rad
    public Translation2d robotVelocity = new Translation2d(); //m/s
    public double robotRotationalVelocity = 0.1; //rad/s
    
    public double timeOfFlight; //seconds
    public double distanceToTarget; //m


    public void newton() {




    }
}
