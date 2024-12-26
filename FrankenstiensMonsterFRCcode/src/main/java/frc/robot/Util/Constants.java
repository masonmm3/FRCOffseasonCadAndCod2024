// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/** Add your docs here. */
public class Constants {
    
  public static final double ROBOT_MASS = (110) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(Units.inchesToMeters(0.233), Units.inchesToMeters(4.547), Units.inchesToMeters(5.868)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(20.1);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class FieldLocations {

    public static final Pose2d RED_AMP = new Pose2d(new Translation2d(14.703, 7.651), new Rotation2d(Units.degreesToRadians(-90)));
    public static final Pose2d RED_SPEAKER = new Pose2d(new Translation2d(16.45, 5.5), new Rotation2d(0));
    public static final Pose2d RED_SOURCE = new Pose2d(new Translation2d(0.781, 0.514), new Rotation2d(Units.degreesToRadians(54)));
    public static final Pose2d RED_PASS_LOCATION = new Pose2d(new Translation2d(15.745, 7.416), new Rotation2d(Units.degreesToRadians(0)));


    public static final Pose2d BLUE_SPEAKER = new Pose2d(new Translation2d(0.05, 5.5), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d BLUE_AMP = new Pose2d(new Translation2d(1.838, 7.651), new Rotation2d(Units.degreesToRadians(-90)));
    public static final Pose2d BLUE_SOURCE = new Pose2d(new Translation2d(15.569, 0.514), new Rotation2d(Units.degreesToRadians(120)));
    public static final Pose2d BLUE_PASS_LOCATION = new Pose2d(new Translation2d(0.957, 7.416), new Rotation2d(Units.degreesToRadians(180)));

  }

  public static final class ShooterConstants {
    public static final double IDLE_RPM = 3000;
  }

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
