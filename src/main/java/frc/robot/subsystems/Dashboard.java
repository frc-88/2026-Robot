// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    double gameTimeInt = Math.floor(gameTime); // Round the seconds down
    String gameTimeString = String.valueOf(Math.floor(gameTimeInt / 60)) + insertZero(String.valueOf(gameTimeInt % 60));

    SmartDashboard.putString("Match Time", gameTimeString);


    if(gameData.length() > 0)
    {
      if (gameTime > 130) { // ---------- TRANSITION SHIFT ----------
        SmartDashboard.putBoolean("Hub Active", true);
        SmartDashboard.putString("Current Period", "TRANSITION SHIFT");
        SmartDashboard.putNumber("Period Time", gameTimeInt-130);

      } else if (gameTime > 105) { // ---------- SHIFT 1 ----------
        SmartDashboard.putString("Current Period", "SHIFT 1");
        SmartDashboard.putNumber("Period Time", gameTimeInt-105);
        switch (gameData.charAt(0))
        {
          case 'B' : //Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) { SmartDashboard.putBoolean("Hub Active", true);
            } else { SmartDashboard.putBoolean("Hub Active", false); }
            break;
          case 'R' : //Red case code (red won autonomous, so blue goes first)
          if (weAreRed) { SmartDashboard.putBoolean("Hub Active", false);
            } else { SmartDashboard.putBoolean("Hub Active", true); }
            break;
          default : //This is corrupt data
            SmartDashboard.putBoolean("Hub Active", true);
            break;
        }
      }

      else if (gameTime > 80) { // ---------- SHIFT 2 ----------
        SmartDashboard.putString("Match Period", "SHIFT 2");
        SmartDashboard.putNumber("Period Time", gameTimeInt-80);
        switch (gameData.charAt(0))
        {
          case 'B' : //Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) { SmartDashboard.putBoolean("Hub Active", false);
            } else { SmartDashboard.putBoolean("Hub Active", true); }
            break;
          case 'R' : //Red case code (red won autonomous, so blue goes first)
          if (weAreRed) { SmartDashboard.putBoolean("Hub Active", true);
            } else { SmartDashboard.putBoolean("Hub Active", false); }
            break;
          default : //This is corrupt data
            SmartDashboard.putBoolean("Hub Active", true);
            break;
        }
      }

      else if (gameTime > 55) { // ---------- SHIFT 3 ----------
        SmartDashboard.putString("Current Period", "SHIFT 3");
        SmartDashboard.putNumber("Period Time", gameTimeInt-5);
        switch (gameData.charAt(0))
        {
          case 'B' : //Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) { SmartDashboard.putBoolean("Hub Active", true);
            } else { SmartDashboard.putBoolean("Hub Active", false); }
            break;
          case 'R' : //Red case code (red won autonomous, so blue goes first)
          if (weAreRed) { SmartDashboard.putBoolean("Hub Active", false);
            } else { SmartDashboard.putBoolean("Hub Active", true); }
            break;
          default : //This is corrupt data
            SmartDashboard.putBoolean("Hub Active", true);
            break;
        }
      }

      else if (gameTime > 30) { // ---------- SHIFT 4 ----------
        SmartDashboard.putString("Match Period", "SHIFT 4");
        SmartDashboard.putNumber("Period Time", gameTimeInt-30);
        switch (gameData.charAt(0))
        {
          case 'B' : //Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) { SmartDashboard.putBoolean("Hub Active", false);
            } else { SmartDashboard.putBoolean("Hub Active", true); }
            break;
          case 'R' : //Red case code (red won autonomous, so blue goes first)
          if (weAreRed) { SmartDashboard.putBoolean("Hub Active", true);
            } else { SmartDashboard.putBoolean("Hub Active", false); }
            break;
          default : //This is corrupt data
            SmartDashboard.putBoolean("Hub Active", true);
            break;
        }
      }

      else { // ---------- END GAME ----------
        SmartDashboard.putString("Match Period", "END GAME");
        SmartDashboard.putNumber("Period Time", gameTimeInt);
        SmartDashboard.putBoolean("Hub Active", true);
      }


    } else { //Code for no data received yet
      SmartDashboard.putBoolean("Hub Active", true);
      if (gameTime < 20) { // In autonomous
        SmartDashboard.putString("Current Period", "AUTONOMOUS");
        SmartDashboard.putNumber("Period Time", gameTimeInt);
      } else { // Probably in transition shift
        SmartDashboard.putString("Current Period", "TRANSITION SHIFT");
        SmartDashboard.putNumber("Period Time", gameTimeInt-130);
      }
    }
  }

  private String insertZero(String startNumber) { // If a number (in string format) is single-digit, insert a zero to make it two digits.
    if (startNumber.length() < 2) {
      return "0" + startNumber;
    }
    return startNumber;
  }
}
