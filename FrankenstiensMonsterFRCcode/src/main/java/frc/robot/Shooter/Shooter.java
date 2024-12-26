// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Swerve.Swerve;
import frc.robot.Util.Constants.FieldLocations;
import frc.robot.Util.GeometryMath;

/** Add your docs here. */
public class Shooter {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private Pose3d shooterAngle;
    private GeometryMath geoMath = new GeometryMath();

    public Shooter(ShooterIO io) {
        this.io = io;

        shooterAngle = new Pose3d(new Translation3d(-0.18,0.15,0.15), new Rotation3d(0,0,0));
    }

    public void ShooterPeriodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("Shooter", inputs);

        shooterAngle = new Pose3d(new Translation3d(-0.18,0.15,0.15), new Rotation3d(0,Units.degreesToRadians(-io.getShooterAngle()),0));

        Logger.recordOutput("Shooter/Angle", shooterAngle);
    }

    public void SmartTeleop(double distanceToPass, double distanceToSpeaker, double distanaceToSource, boolean hasGamePiece, boolean shoot) {
            if (distanceToSpeaker > Units.inchesToMeters(240) && shoot) {
                io.AimAtPass(distanceToPass);
                io.FlywheelSpeedUpPass(distanceToPass);
            } else if (distanceToSpeaker < Units.inchesToMeters(240) && shoot) {
                io.AimAtSpeaker(distanceToSpeaker);
                io.FlywheelSpeedUpSpeaker(distanceToSpeaker);
            } else if (distanceToPass > Units.inchesToMeters(225) && distanaceToSource > Units.inchesToMeters(170) && hasGamePiece) {
                io.AimAtPass(distanceToPass);
                io.FlywheelSpeedUpPass(distanceToPass);
            } else if (distanceToSpeaker < Units.inchesToMeters(190) && hasGamePiece) {
                io.AimAtSpeaker(distanceToSpeaker);
                io.FlywheelSpeedUpSpeaker(distanceToSpeaker);
            } else {
                io.StowShooter();
                io.IdleSpin();
            }
    }

    public void autonomous(Swerve swerve) {

        Pose2d speedAdjustedPose;

        if (swerve.isRedAlliance()) {
            speedAdjustedPose = swerve.getPose().plus(new Transform2d(new Translation2d(swerve.getSwerve().getFieldVelocity().vxMetersPerSecond, swerve.getSwerve().getFieldVelocity().vyMetersPerSecond), new Rotation2d(swerve.getSwerve().getFieldVelocity().omegaRadiansPerSecond)).div(2.8));
            io.AimAtSpeaker(geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, speedAdjustedPose));
            io.FlywheelSpeedUpSpeaker(geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, speedAdjustedPose));
        } else {
            speedAdjustedPose = swerve.getPose().plus(new Transform2d(new Translation2d(swerve.getSwerve().getFieldVelocity().vxMetersPerSecond, swerve.getSwerve().getFieldVelocity().vyMetersPerSecond), new Rotation2d(swerve.getSwerve().getFieldVelocity().omegaRadiansPerSecond)).div(-2.8));
            io.AimAtSpeaker(geoMath.distanceToPoint(FieldLocations.BLUE_SPEAKER, speedAdjustedPose));
            io.FlywheelSpeedUpSpeaker(geoMath.distanceToPoint(FieldLocations.BLUE_SPEAKER, speedAdjustedPose));
        }
    }

    public double ArmError() {
        return io.getArmError();
    }

    public Pose3d getArmPose() {
        return shooterAngle;
    }

    public double FlywheelError() {
        return io.getShooterError();
    }

    public double FlywheelSpeed() {
        return io.gettShooterSpeeds();
    }

    public Rotation2d getArmAngle() {
        return new Rotation2d(Units.degreesToRadians(io.getShooterAngle()));
    }

    public void simReset() {
        io.reset();
    }
}
