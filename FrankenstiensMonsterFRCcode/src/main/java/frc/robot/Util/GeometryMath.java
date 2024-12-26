// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class GeometryMath {
    public GeometryMath() {}

    public Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Units.degreesToRadians(degrees));
    }

    public double distanceToPoint(Pose2d point, Pose2d currentPose) {
        return Math.sqrt((Math.pow(currentPose.getX() - point.getX(), 2)) + Math.pow(currentPose.getY() - point.getY(), 2));
    }
}
