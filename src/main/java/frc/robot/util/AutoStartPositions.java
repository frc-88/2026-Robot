// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Constants for the starting positions of autos. This is used to calculate if the starting position
 * for the match is safe. This file should be updated whenever autos are changed.
 */
public class AutoStartPositions {
  public Pose2d getStartingPose(String autoName) {
    Pose2d startingPose = new Pose2d(0, 0, Rotation2d.kZero);

    switch (autoName) {
      case "Double Chompei Left":
        startingPose = new Pose2d(4.3877651515151515, 7.4, Rotation2d.kZero);
        break;

      case "Double Chompei Right":
        startingPose = new Pose2d(4.3877651515151515, 0.6222979797979815, Rotation2d.kZero);
        break;

      case "SimpleCenter":
        startingPose = new Pose2d(3.585746835443037, 4.035, Rotation2d.kCCW_90deg);
        break;

      case "Double Bump OP Left":
        startingPose = new Pose2d(4.350, 7.490, Rotation2d.kCW_90deg);
        break;

      case "Shallow Double Bump OP Left":
        startingPose = new Pose2d(4.350, 7.490, Rotation2d.kCW_90deg);
        break;

      case "Double Bump OP Right":
        startingPose = new Pose2d(4.388, 0.622, Rotation2d.kCCW_90deg);
        break;

      case "Shallow Double Bump OP Right":
        startingPose = new Pose2d(4.388, 0.622, Rotation2d.kCCW_90deg);
        break;

      case "SimpleCenterDepot":
        startingPose = new Pose2d(3.585746835443037, 4.035, Rotation2d.kCCW_90deg);
        break;

      case "CenterDepot & Outpost":
        startingPose = new Pose2d(3.585746835443037, 4.035, Rotation2d.kCCW_90deg);
        break;

      case "Left Follow The Leader Nutrons":
        startingPose = new Pose2d(3.547, 7.406, Rotation2d.kZero);
        break;
      case "Right Follow The Leader Nutrons":
        startingPose = new Pose2d(3.560, 0.664, Rotation2d.kZero);
        break;
      case "Left Follow The Leader Wyndham":
        startingPose = new Pose2d(3.547, 7.406, Rotation2d.kZero);
        break;
      case "Right Follow The Leader Wyndham":
        startingPose = new Pose2d(3.560, 0.664, Rotation2d.kZero);
        break;

      default:
        break;
    }

    return Util.flipIfRed(startingPose);
  }
}
