// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import frc.robot.Shooter.Shooter;
import frc.robot.Swerve.Swerve;

/** Add your docs here. */
public class Indexer {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private Shooter shooter;
    private Swerve swerve;

    public Indexer(IndexerIO io, Shooter shooter, Swerve swerve) {
        this.io = io;
        this.shooter = shooter;
        this.swerve = swerve;
    }

     public void IndexerPeriodic() {
        io.updateInputs(inputs);
        
        Logger.processInputs("Indexer", inputs);
    }

    public void IntakeAndIndex(boolean Intaking) {
            io.indexGamePiece(Intaking);
    }

    public boolean gamePieceInShooter() {
        return io.gamePieceInShooter();
    }

    public void Shoot(boolean shoot) {
        if(shooter.FlywheelError() < 100 && shooter.ArmError() < 10 && io.gamePieceInShooter() && shoot && swerve.SmartAimError() < Units.degreesToRadians(1)) {
            io.shootGamePiece(shooter.FlywheelSpeed(), shooter.getArmAngle(), swerve.getPose(), swerve.getSwerve().getFieldVelocity());
        }
    }

    public void ShootAuto(boolean shoot) {
        if(shooter.FlywheelError() < 300 && shooter.ArmError() < 1 && io.gamePieceInShooter() && shoot) {
            io.shootGamePiece(shooter.FlywheelSpeed(), shooter.getArmAngle(), swerve.getPose(), swerve.getSwerve().getFieldVelocity());
        }
    }

    public void reset() {
        io.reset();
        shooter.simReset();
    }
}
