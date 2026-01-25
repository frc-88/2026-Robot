// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {

  public Feeder feeder = new Feeder();
  public Shooter shooter = new Shooter();
  public Intake intake = new Intake();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData("RunFeeder", feeder.runFeeder());
    SmartDashboard.putData("StopFeeder", feeder.stopFeeder());
    SmartDashboard.putData("RunShooter", shooter.runShooter());
    SmartDashboard.putData("StopShooter", shooter.stopShooter());
    feeder.setDefaultCommand(feeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    SmartDashboard.putData("RunIntake", intake.runIndexer());
    SmartDashboard.putData("StopIntake", intake.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
