// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.Intake;
import frc.robot.Commands.ShootAtSpeaker;
import frc.robot.Indexer.Indexer;
import frc.robot.Indexer.IndexerHardware;
import frc.robot.Indexer.IndexerSim;
import frc.robot.Shooter.Shooter;
import frc.robot.Shooter.ShooterSim;
import frc.robot.Swerve.Swerve;
import frc.robot.Swerve.SwerveHardware;

/** Add your docs here. */
public class RobotContainer {

    public static Swerve swerve;

    public static Indexer indexer;

    public static Shooter shooter;

    public final LoggedDashboardChooser<Command> autoChooser;

    public Command shoot;

    public Command intake;


    public RobotContainer() {

        if (Robot.isSimulation()) {
            swerve = new Swerve(new SwerveHardware());
            shooter = new Shooter(new ShooterSim());
            indexer = new Indexer(new IndexerSim(swerve, shooter), shooter, swerve);
        } else {
            swerve = new Swerve(new SwerveHardware());
            shooter = new Shooter(new ShooterSim());
            indexer = new Indexer(new IndexerHardware(), shooter, swerve);
        }

        shoot = new ShootAtSpeaker(indexer);
        intake = new Intake(indexer);

        NamedCommands.registerCommand("intake", intake);
        NamedCommands.registerCommand("shoot", shoot);

        swerve.setupPathPlanner();

        autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser()); // create choosable dash value.
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
