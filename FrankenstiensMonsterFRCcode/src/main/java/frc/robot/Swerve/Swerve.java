// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision.Vision;
import swervelib.SwerveDrive;

/** Add your docs here. */
public class Swerve {
    private final SwerveIO io;
    private final SwerveIOInputsAutoLogged inputs = new SwerveIOInputsAutoLogged();

    private Vision _vision;
    private PIDController pidController = new PIDController(8.0, 0, 0.0);
    private boolean notAiming;

    private Trajectory trajectory;

    public Swerve(SwerveIO io) {
        this.io = io;

        setupPathPlanner();

        setupPhotonVision();

        pidController.enableContinuousInput(Units.degreesToRadians(-180), Units.degreesToRadians(180));

        notAiming = true;
    }

    public void SwervePeriodic() {
        io.updateInputs(inputs);

        io.updateOdometry();

        io.getSwerve().postTrajectory(trajectory);

        _vision.updatePoseEstimation(io.getSwerve());

        Logger.processInputs("Swerve/MotorInfo", inputs);

        Logger.recordOutput("Odometry/Robot", io.getPose());

        Logger.recordOutput("Odometry/Sim", io.getSimPose());

        Logger.recordOutput("Odometry/lastVision", _vision.ReturnPhotonPose());

        Logger.recordOutput( "Swerve Drive/States", io.getModuleState());

        Logger.recordOutput("Swerve Drive/Trajectory", trajectory);
    }

    public void fieldRelativeTeleop(double LeftX, double LeftY, double RightX, double steerSens) {
        ChassisSpeeds desiredSpeeds = io.getTargetSpeeds(LeftY, LeftX, new Rotation2d(RightX * Math.PI));
        desiredSpeeds.omegaRadiansPerSecond = RightX * Math.PI * steerSens;

        io.driveFieldOriented(desiredSpeeds);
    }

    public void fieldRelativeTeleopSmartAim(double LeftX, double LeftY, double RightX, double steerSens, boolean aimToPoint, boolean rotateToAngle, boolean assistive,  Pose2d aimPose, boolean slow, boolean lockPose, boolean zeroGyro) {
        ChassisSpeeds desiredSpeeds = io.getTargetSpeeds(LeftY, LeftX, new Rotation2d(RightX * Math.PI));

        if (aimToPoint) {
            desiredSpeeds.omegaRadiansPerSecond =  pidController.calculate(io.getPose().plus(new Transform2d(new Translation2d(io.getFieldVelocity().vxMetersPerSecond, io.getFieldVelocity().vyMetersPerSecond), new Rotation2d(io.getFieldVelocity().omegaRadiansPerSecond)).div(2.8)).getRotation().minus(aimPose.getRotation()).getRadians(), getPointYaw(aimPose).getRadians());   
            notAiming = false;
        } else if (rotateToAngle) {
            desiredSpeeds.omegaRadiansPerSecond = pidController.calculate(io.getPose().getRotation().getRadians(), new Rotation2d(Units.degreesToRadians(aimPose.getRotation().getDegrees())).getRadians()); 
            notAiming = false;
        }else {
            desiredSpeeds.omegaRadiansPerSecond = RightX * Math.PI * steerSens;
            notAiming = true;
        }
        
        if (assistive) {
            desiredSpeeds.omegaRadiansPerSecond = (desiredSpeeds.omegaRadiansPerSecond/(1+(5*(Math.abs(RightX*Math.PI*steerSens))))) + (RightX*Math.PI*steerSens);
        }

        if (slow) {
            desiredSpeeds.vxMetersPerSecond *= 0.4;
            desiredSpeeds.vyMetersPerSecond *= 0.4;
            desiredSpeeds.omegaRadiansPerSecond *= 3;
        }

        if (lockPose) {
            io.moduleLock();
        } else {
            io.driveFieldOriented(desiredSpeeds);
        }

        if (zeroGyro) {
            io.zeroGyroWithAlliance();
        }
    }

    public double SmartAimError() {
        if (notAiming) {
            return 90;
        } else {
            return Math.abs(pidController.getError());
        }
        
    }

    public void setupPhotonVision()
    {
        //camera settings are on line 335 of Vision.java
        _vision = new Vision(io::getPose, io.getField());
    }

    public void simReset() {
        io.getSwerve().getMapleSimDrive().get().clearAccumulatedForce();
        io.getSwerve().getMapleSimDrive().get().clearAccumulatedTorque();
        io.getSwerve().getMapleSimDrive().get().clearForce();
        io.getSwerve().getMapleSimDrive().get().clearTorque();
        io.getSwerve().getMapleSimDrive().get().setAngularVelocity(0);
        io.getSwerve().getMapleSimDrive().get().setLinearVelocity(0, 0);
        io.getSwerve().getMapleSimDrive().get().setAtRest(true);
        io.getSwerve().setChassisSpeeds(new ChassisSpeeds());
    }

    public void setupPathPlanner()
    {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try
        {
          config = RobotConfig.fromGUISettings();

          final boolean enableFeedforward = true;
          // Configure AutoBuilder last
           AutoBuilder.configure(
                io::getPose,
                // Robot pose supplier
                io::resetOdometry,
                // Method to reset odometry (will be called if your auto has a starting pose)
                io::getRobotVelocity,
                // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speedsRobotRelative, moduleFeedForwards) -> {
                    if (enableFeedforward)
                    {
                        io.drivePathPlanner(
                            speedsRobotRelative,
                            io.getKinematics().toSwerveModuleStates(speedsRobotRelative),
                            moduleFeedForwards.linearForces()
                        );
                    } 
                    else
                    {
                        io.setChassisSpeeds(speedsRobotRelative);
                    }
                },
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController(
                    // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(9.0, 0.0, 0.1),
                    // Translation PID constants
                    new PIDConstants(9.0, 1.0, 0.25)
                    // Rotation PID constants
                ),
            config,
            // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            new SwerveSub()
            // Reference to dummy subsystem to set requirements
            );

        } 
        catch (Exception e)
        {
            // Handle exception as needed
            e.printStackTrace();
        }

        //Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));});//loging stuff
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);});//logging stuff
        Pathfinding.setPathfinder(new LocalADStar());
        PathfindingCommand.warmupCommand().schedule();
    }

    public Command driveToPose(Pose2d pose)
    {
// Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        io.getSwerve().getMaximumChassisVelocity(), 4.0,
        io.getSwerve().getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

// Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
                                     );
    }

    public Rotation2d getPointYaw(Pose2d point)
    {
        double x = io.getPose().getX() - point.getX();
        double y = io.getPose().getY() - point.getY();

        return new Rotation2d(Math.atan(y/x));
    }

    public boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    public void setBrake(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    public Pose2d getPose() {
        return io.getPose();
    }

    public SwerveDrive getSwerve() {
        return io.getSwerve();
    }

    /**
     * 
     * @param autoName
     * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
     */
    public Command getAutonomousCommand(String autoName)
    {
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return new PathPlannerAuto(autoName);
    }

}
