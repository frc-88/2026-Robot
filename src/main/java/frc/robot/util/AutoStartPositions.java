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
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

    switch (autoName) {
      case "Chompei Left & Depot":
        startingPose = new Pose2d(4.3877651515151515, 7.4, new Rotation2d());
        break;

      case "Double Chompei Left":
        startingPose = new Pose2d(4.3877651515151515, 7.4, new Rotation2d());
        break;

      case "Fast Chompei Left & Depot":
        startingPose = new Pose2d(4.3877651515151515, 7.4, new Rotation2d());
        break;

      case "Chompei Right & Outpost":
        startingPose = new Pose2d(4.3877651515151515, 0.6222979797979815, new Rotation2d());
        break;

      case "Double Chompei Right":
        startingPose = new Pose2d(4.3877651515151515, 0.6222979797979815, new Rotation2d());
        break;

      case "Fast Chompei Right & Outpost":
        startingPose = new Pose2d(4.3877651515151515, 0.6222979797979815, new Rotation2d());
        break;

      case "Double Bump Chompei Left":
        startingPose = new Pose2d(4.3877651515151515, 7.4, new Rotation2d());
        break;

      case "Double Bump Chompei Right":
        startingPose = new Pose2d(4.3877651515151515, 0.6222979797979815, new Rotation2d());
        break;

      case "SimpleCenter":
        startingPose = new Pose2d(3.585746835443037, 4.035, new Rotation2d());
        break;

      default:
        break;
    }

    return startingPose;
  }
}
