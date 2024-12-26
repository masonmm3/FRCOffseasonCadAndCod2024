// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Indexer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Add your docs here. */
public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        public Pose3d gamePieceLocation = new Pose3d();
        public String gamePieceState = "none";
        public double gamePiecesShot = 0;
        public double gamePiecesMade = 0;
    }

    public default boolean gamePieceInShooter() {
        return false;
    }

    public default void indexGamePiece(boolean running) {}

    public default void shootGamePiece(double velocityRPM, Rotation2d shooterRotation, Pose2d RobotPose, ChassisSpeeds chassisSpeedsFieldRelative) {}

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void reset() {}
    
}
