// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Prototype;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {

  public Prototype prototype = new Prototype();
  public Turret turret = new Turret();
  public Feeder feeder = new Feeder();
  public Shooter shooter = new Shooter();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    SmartDashboard.putData("run", prototype.runIndexer());
    SmartDashboard.putData("stop", prototype.stop());
    SmartDashboard.putData("RunFeeder", feeder.runFeeder());
    SmartDashboard.putData("StopFeeder", feeder.stopFeeder());
    SmartDashboard.putData("RunShooter", shooter.runShooter());
    SmartDashboard.putData("StopShooter", shooter.stopShooter());
    turret.setDefaultCommand(turret.stopAllTurret());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
