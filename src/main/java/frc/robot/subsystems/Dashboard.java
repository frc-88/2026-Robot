// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Util;

public class Dashboard extends SubsystemBase {
  /** Creates a new Dashboard. */
  private double periodTimeRemaining = 0;
  private boolean isHubActive = true;

  public Dashboard() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    String gameData = DriverStation.getGameSpecificMessage();
    boolean weAreRed = Util.weAreRed();
    double gameTime = DriverStation.getMatchTime();

    double gameTimeInt = Math.ceil(gameTime); // Round the seconds down
    double gameMinsInt = gameTimeInt / 60;
    String gameTimeString =
        String.valueOf(Math.floor(gameMinsInt))
            + ":"
            + insertZero(String.valueOf(gameTimeInt % 60));

    // gameTimeString = String.format("%1$:%2$02.0", gameMinsInt); // Not sure how to use this
    // properly

    SmartDashboard.putString("Driver Dashboard/Match Time", gameTimeString);

    String currentMatchPeriod = "unknown";

    if (gameData.length() > 0) {
      if (gameTimeInt < 0) {
        isHubActive = false;
        currentMatchPeriod = "DISABLED";
        periodTimeRemaining = gameTimeInt;
      } else if (gameTimeInt >= 130) { // ---------- TRANSITION SHIFT ----------
        isHubActive = true;
        currentMatchPeriod = "TRANSITION SHIFT";
        periodTimeRemaining = gameTimeInt - 130;

      } else if (gameTimeInt >= 105) { // ---------- SHIFT 1 ----------
        currentMatchPeriod = "SHIFT 1";
        periodTimeRemaining = gameTimeInt - 105;
        switch (gameData.charAt(0)) {
          case 'B': // Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) {
              isHubActive = true;
            } else {
              isHubActive = false;
            }
            break;
          case 'R': // Red case code (red won autonomous, so blue goes first)
            if (weAreRed) {
              isHubActive = false;
            } else {
              isHubActive = true;
            }
            break;
          default: // This is corrupt data
            isHubActive = true;
            break;
        }
      } else if (gameTimeInt >= 80) { // ---------- SHIFT 2 ----------
        currentMatchPeriod = "SHIFT 2";
        periodTimeRemaining = gameTimeInt - 80;
        switch (gameData.charAt(0)) {
          case 'B': // Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) {
              isHubActive = false;
            } else {
              isHubActive = true;
            }
            break;
          case 'R': // Red case code (red won autonomous, so blue goes first)
            if (weAreRed) {
              isHubActive = true;
            } else {
              isHubActive = false;
            }
            break;
          default: // This is corrupt data
            isHubActive = true;
            break;
        }
      } else if (gameTimeInt >= 55) { // ---------- SHIFT 3 ----------
        currentMatchPeriod = "SHIFT 3";
        periodTimeRemaining = gameTimeInt - 55;
        switch (gameData.charAt(0)) {
          case 'B': // Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) {
              isHubActive = true;
            } else {
              isHubActive = false;
            }
            break;
          case 'R': // Red case code (red won autonomous, so blue goes first)
            if (weAreRed) {
              isHubActive = false;
            } else {
              isHubActive = true;
            }
            break;
          default: // This is corrupt data
            isHubActive = true;
            break;
        }
      } else if (gameTimeInt >= 30) { // ---------- SHIFT 4 ----------
        currentMatchPeriod = "SHIFT 4";
        periodTimeRemaining = gameTimeInt - 30;
        switch (gameData.charAt(0)) {
          case 'B': // Blue case code (blue won autonomous, so red goes first)
            if (weAreRed) {
              isHubActive = false;
            } else {
              isHubActive = true;
            }
            break;
          case 'R': // Red case code (red won autonomous, so blue goes first)
            if (weAreRed) {
              isHubActive = true;
            } else {
              isHubActive = false;
            }
            break;
          default: // This is corrupt data
            isHubActive = true;
            break;
        }
      } else if (gameTimeInt < 30
          && !DriverStation.isAutonomous()) { // ---------- END GAME ----------
        currentMatchPeriod = "END GAME";
        periodTimeRemaining = gameTimeInt;
        isHubActive = true;
      } else if (gameTimeInt <= 20 && DriverStation.isAutonomous()) {
        currentMatchPeriod = "AUTONOMOUS";
        periodTimeRemaining = gameTimeInt;
        isHubActive = true;
      } else {
        currentMatchPeriod = "IDK BRUH";
        isHubActive = false;
        periodTimeRemaining = -1;
      }

    } else { // Code for no data received yet
      isHubActive = false;
      currentMatchPeriod = "DISABLED";
      isHubActive = false;
      periodTimeRemaining = -1;
    }

    SmartDashboard.putBoolean("Driver Dashboard/Hub Active", isHubActive);
    SmartDashboard.putString("Driver Dashboard/Current Match Period", currentMatchPeriod);
    SmartDashboard.putNumber("Driver Dashboard/Period Time", periodTimeRemaining);
    SmartDashboard.putString("Driver Dashboard/GameData", gameData);
  }

  public double getPeriodTimeRemaining() {
    return periodTimeRemaining;
  }

  public boolean getIsHubActive() {
    return isHubActive;
  }

  private String insertZero(
      String
          startNumber) { // If a number (in string format) is single-digit, insert a zero to make it
    // two digits.
    if (startNumber.length() < 2) {
      return "0" + startNumber;
    }
    return startNumber;
  }
}
