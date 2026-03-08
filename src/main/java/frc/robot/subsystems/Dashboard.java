// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  public Dashboard() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String gameData = DriverStation.getGameSpecificMessage();
    boolean weAreRed = DriverStation.getAlliance().equals(Alliance.Red);
    double gameTime = DriverStation.getMatchTime();

if(gameData.length() > 0)
{
  switch (gameData.charAt(0))
  {
    case 'B' :
      //Blue case code
      break;
    case 'R' :
      //Red case code
      break;
    default :
      //This is corrupt data
      break;
  }
} else {
  //Code for no data received yet
}
  }
}
