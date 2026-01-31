// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.HopperFeeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFeeder;
import frc.robot.subsystems.Spinner;

public class RobotContainer {

  public HopperFeeder hopperFeeder = new HopperFeeder();
  public ShooterFeeder shooterFeeder = new ShooterFeeder();
  public Shooter shooter = new Shooter();
  public Intake intake = new Intake();
  public Spinner spinner = new Spinner();

  public RobotContainer() {
    configureBindings();
  }
  
  private void configureBindings() {
    SmartDashboard.putData("RunHopperFeeder", hopperFeeder.runFeeder());
    SmartDashboard.putData("StopHopperFeeder", hopperFeeder.stopFeeder());
    SmartDashboard.putData("RunShooter", shooter.runShooter());
    SmartDashboard.putData("StopShooter", shooter.stopShooter());
    SmartDashboard.putData("RunShooterFeeder", shooterFeeder.runFeeder());
    SmartDashboard.putData("StopShooterFeeder", shooterFeeder.stopFeeder());
    SmartDashboard.putData("RunShooter", shooter.runShooter());
    SmartDashboard.putData("StopShooter", shooter.stopShooter());
    SmartDashboard.putData("RunIntake", intake.runIndexer());
    SmartDashboard.putData("StopIntake", intake.stopIntake());
    SmartDashboard.putData("RunSpinner", spinner.runSpinner());
    SmartDashboard.putData("StopSpinner", spinner.stopSpinner());
    SmartDashboard.putData("RunHopper", hopperFeeder.runFeeder().alongWith(spinner.runSpinner()));
    SmartDashboard.putData("StopHopper", hopperFeeder.stopFeeder().alongWith(spinner.stopSpinner()));
    SmartDashboard.putData("RunFooter", shooterFeeder.runFeeder().alongWith(shooter.runShooter()));
    SmartDashboard.putData("StopFooter", shooterFeeder.stopFeeder().alongWith(shooter.stopShooter()));
    hopperFeeder.setDefaultCommand(hopperFeeder.stopFeeder());
    shooter.setDefaultCommand(shooter.stopShooter());
    spinner.setDefaultCommand(spinner.stopSpinner());
    intake.setDefaultCommand(intake.stopIntake());
    shooterFeeder.setDefaultCommand(shooterFeeder.stopFeeder());
    SmartDashboard.putData("RunShooterVoltage", shooter.runShooterVoltage());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
