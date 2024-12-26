// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Indexer;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Shooter.Shooter;
import frc.robot.Swerve.Swerve;

/** Add your docs here. */
public class IndexerSim implements IndexerIO {
    private final IntakeSimulation _intakeSimulation;

    private boolean gamePieceInRobot;
    private boolean intakeHasGamePiece;
    private boolean indexerHasGamePiece;
    private boolean shooterHasGamePiece;
    private Swerve swerve;
    private Shooter shooter;

    private double shot;
    private double made;


    private Timer transferTimer;

    private final double transferTime = 0.18;

    public IndexerSim(Swerve swerve, Shooter shooter) {
        this.swerve = swerve;
        AbstractDriveTrainSimulation driveTrain = swerve.getSwerve().getMapleSimDrive().get();
        _intakeSimulation = new IntakeSimulation("Note", driveTrain, Distance.ofRelativeUnits(28, edu.wpi.first.units.Units.Inch),Distance.ofRelativeUnits(4, edu.wpi.first.units.Units.Inch), IntakeSimulation.IntakeSide.BACK ,1);
        driveTrain.addFixture(_intakeSimulation.getShape());
        _intakeSimulation.register(SimulatedArena.getInstance());
        _intakeSimulation.clearGamePiecesToRemoveQueue();
        transferTimer = new Timer();
        gamePieceInRobot = false;
        intakeHasGamePiece = false;
        indexerHasGamePiece = false;
        shooterHasGamePiece = false;
        shot = 0;
        made = 0;
        this.shooter = shooter;
    }

    @Override
    public boolean gamePieceInShooter() {
        return shooterHasGamePiece;
    }

    @Override
    public void reset() {
        gamePieceInRobot = true;
        intakeHasGamePiece = false;
        indexerHasGamePiece = false;
        shooterHasGamePiece = true;
        shot = 0;
        made = 0;
        transferTimer.stop();
        transferTimer.reset();
    }

    @Override
    public void indexGamePiece(boolean running) {

        if (running) {
            _intakeSimulation.startIntake();
        } else {
            _intakeSimulation.stopIntake();
        }

        if (_intakeSimulation.getGamePiecesAmount() != 0 && !gamePieceInRobot) {
            intakeHasGamePiece = true;
            gamePieceInRobot = true;
        } else if(gamePieceInRobot && intakeHasGamePiece) {
            transferTimer.start();
            if (transferTimer.get() > transferTime) {
                intakeHasGamePiece = false;
                indexerHasGamePiece = true;
                transferTimer.stop();
                transferTimer.reset();
            }
        } else if (gamePieceInRobot && indexerHasGamePiece) {
            transferTimer.start();
            if (transferTimer.get() > transferTime) {
                indexerHasGamePiece = false;
                shooterHasGamePiece = true;
                transferTimer.stop();
                transferTimer.reset();
            }
        }
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {

        //x: forward+ , y: Left+, z: up+
        if (gamePieceInRobot && intakeHasGamePiece) {
            inputs.gamePieceState = "Intake";
            inputs.gamePieceLocation = new Pose3d(swerve.getSwerve().getSimulationDriveTrainPose().get()).plus(new Transform3d(-0.42, 0, 0, new Rotation3d(0,90,0)));
        } else if (gamePieceInRobot && indexerHasGamePiece) {
            inputs.gamePieceState = "Indexer";
            inputs.gamePieceLocation = new Pose3d(swerve.getSwerve().getSimulationDriveTrainPose().get()).plus(new Transform3d(-0.27, 0, 0.18, new Rotation3d()));
        } else if (gamePieceInRobot && shooterHasGamePiece) {
            inputs.gamePieceState = "Shooter";
            inputs.gamePieceLocation = new Pose3d(swerve.getSwerve().getSimulationDriveTrainPose().get()).plus(new Transform3d(0.05, -0.13, 0.12, new Rotation3d())).plus(new Transform3d(shooter.getArmPose().getTranslation(),shooter.getArmPose().getRotation().minus(new Rotation3d(0,Units.degreesToRadians(15),0))));
        } else {
            inputs.gamePieceState = "none";
            inputs.gamePieceLocation = new Pose3d(swerve.getSwerve().getSimulationDriveTrainPose().get()).plus(new Transform3d(0, 0, -1, new Rotation3d()));
        }

        inputs.gamePiecesMade = made;
        inputs.gamePiecesShot = shot;

    }

    @Override
    public void shootGamePiece(double velocityRPM, Rotation2d shooterRotation, Pose2d RobotPose, ChassisSpeeds chassisSpeedsFieldRelative) {
        NoteOnFly noteOnFly = new NoteOnFly(
        // Specify the position of the chassis when the note is launched
        RobotPose.getTranslation(),
        // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
        new Translation2d(0.2, 0),
        // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
        chassisSpeedsFieldRelative,
        // The shooter facing direction is the same as the robot’s facing direction
        RobotPose.getRotation()
                // Add the shooter’s rotation
        ,
        // Initial height of the flying note
        0.45,
        // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
        velocityRPM / 6000 * 35,
        // The angle at which the note is launched
        shooterRotation.getRadians()
        );

        noteOnFly
        // Configure the Speaker (of the current alliance) as the target of the projectile
        .asSpeakerShotNote(() -> made += 1);

        noteOnFly
        // Configure callbacks to visualize the flight trajectory of the projectile
        .withProjectileTrajectoryDisplayCallBack(
        // Callback for when the note will eventually hit the target (if configured)
        (pose3ds) -> Logger.recordOutput("Flywheel/NoteProjectileSuccessfulShot", pose3ds.toArray(Pose3d[]::new)),
        // Callback for when the note will eventually miss the target, or if no target is configured
        (pose3ds) -> Logger.recordOutput("Flywheel/NoteProjectileUnsuccessfulShot", pose3ds.toArray(Pose3d[]::new))
        );

        noteOnFly
        // Configure the note projectile to become a NoteOnField upon touching the ground
        .enableBecomeNoteOnFieldAfterTouchGround();

        // Add the projectile to the simulated arena
        SimulatedArena.getInstance().addGamePieceProjectile(noteOnFly);

        gamePieceInRobot = false;
        shooterHasGamePiece = false;

        _intakeSimulation.obtainGamePieceFromIntake();
        shot += 1;
    }
}
