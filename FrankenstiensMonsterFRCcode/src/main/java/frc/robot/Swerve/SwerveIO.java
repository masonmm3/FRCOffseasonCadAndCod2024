// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveDriveConfiguration;

/** Add your docs here. */
public interface SwerveIO {

    @AutoLog
    public static class SwerveIOInputs {

        //Left Front
        public double LeftFrontModuleAngleEncoderAngle = 0.0;
        public double LeftFrontModuleAngleVelocity = 0.0;
        public double LeftFrontModuleAngleVolt = 0.0;
        public double LeftFrontModuleAngleCurrent = 0.0;
        public double LeftFrontModuleAngleInternalAngle = 0.0;

        public double LeftFrontModuleDrivePosition = 0.0;
        public double LeftFrontModuleDriveVelocity = 0.0;
        public double LeftFrontModuleDriveVolt = 0.0;
        public double LeftFrontModuleDriveCurrent = 0.0;

        //Right Front
        public double RightFrontModuleAngleEncoderAngle = 0.0;
        public double RightFrontModuleAngleVelocity = 0.0;
        public double RightFrontModuleAngleVolt = 0.0;
        public double RightFrontModuleAngleCurrent = 0.0;
        public double RightFrontModuleAngleInternalAngle = 0.0;

        public double RightFrontModuleDrivePosition = 0.0;
        public double RightFrontModuleDriveVelocity = 0.0;
        public double RightFrontModuleDriveVolt = 0.0;
        public double RightFrontModuleDriveCurrent = 0.0;

        // Left Rear
        public double LeftRearModuleAngleEncoderAngle = 0.0;
        public double LeftRearModuleAngleVelocity = 0.0;
        public double LeftRearModuleAngleVolt = 0.0;
        public double LeftRearModuleAngleCurrent = 0.0;
        public double LeftRearModuleAngleInternalAngle = 0.0;

        public double LeftRearModuleDrivePosition = 0.0;
        public double LeftRearModuleDriveVelocity = 0.0;
        public double LeftRearModuleDriveVolt = 0.0;
        public double LeftRearModuleDriveCurrent = 0.0;

        public double RightRearModuleAngleEncoderAngle = 0.0;
        public double RightRearModuleAngleVelocity = 0.0;
        public double RightRearModuleAngleVolt = 0.0;
        public double RightRearModuleAngleCurrent = 0.0;
        public double RightRearModuleAngleInternalAngle = 0.0;

        public double RightRearModuleDrivePosition = 0.0;
        public double RightRearModuleDriveVelocity = 0.0;
        public double RightRearModuleDriveVolt = 0.0;
        public double RightRearModuleDriveCurrent = 0.0;
    }

    public default void updateInputs(SwerveIOInputs inputs) {}

    public default void driveChasisSpeeds(ChassisSpeeds velocity) {}

    public default void driveTeleop(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {}

    public default void drive(Translation2d translation, double rotation, boolean fieldRelative) {}

    public default void driveFieldOriented(ChassisSpeeds velocity) {}

    public  default void drivePathPlanner(ChassisSpeeds speeds, SwerveModuleState[] modules, Force[] forwardForce) {}

    public default  void resetOdometry(Pose2d initialHolonomicPose) {}

    public default void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {}

    public default void setBrakeMode(boolean enabled) {}

    public default void updateOdometry() {}

    public default void zeroGyro() {}

    public default void moduleLock() {}

    public default boolean isRedAlliance() {
        return false;
    }

    public default ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        return null;
    }

    public default Pose2d getPose() {
        return null;
    }

    public default Pose2d getSimPose() {
        return null;
    }

    public default SwerveModuleState[] getModuleState() {
        return null;
    }

    public default SwerveModuleState[] getSetpoint() {
        return null;
    }

    public default Field2d getField() {
        return null;
    }

    public default Rotation2d getHeading() {
        return null;
    }

    public default SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return null;
    }

    public default ChassisSpeeds getFieldVelocity() {
        return null;
    }

    public default ChassisSpeeds getRobotVelocity() {
        return null;
    }

    public default SwerveDriveKinematics getKinematics() {
        return null;
    }

    public default SwerveDrive getSwerve() {
        return null;
    }

    public default void zeroGyroWithAlliance() {}
} 
