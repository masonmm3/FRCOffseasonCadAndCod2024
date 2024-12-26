// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.GeometryMath;
import frc.robot.Util.Constants.FieldLocations;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private RobotContainer _robotContainer;
  private Command _autonomousCommand;
  private XboxController driver;
  private GeometryMath geoMath = new GeometryMath();
  private Pose2d aimPoint;
  private boolean aiming;
  private boolean driving;
  private boolean wasDriving;
  private boolean rotate;
  private boolean assistive;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {

    driver = new XboxController(0);

     if (Robot.isReal()) {
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
     } else if (Robot.isSimulation()) {
        Logger.addDataReceiver(new NT4Publisher());
     } else {
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
     }
      
    Logger.start();

    _robotContainer = new RobotContainer();

    SimulatedArena.getInstance();
        
    wasDriving = false;
  }

  @Override 
  public void robotInit() {
    RobotContainer.swerve.setupPathPlanner();
  }

  @Override
  public void robotPeriodic() {
      // Runs the Scheduler. This is responsible for polling buttons, adding
      // newly-scheduled commands, running already-scheduled commands, removing
      // finished or interrupted commands, and running subsystem periodic() methods.
      // This must be called from the robot's periodic block in order for anything in
      // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    RobotContainer.swerve.SwervePeriodic();
    RobotContainer.indexer.IndexerPeriodic();
    RobotContainer.shooter.ShooterPeriodic();

    Logger.recordOutput("Odometry/FieldInfo/RedSouceDistance", Units.metersToInches(geoMath.distanceToPoint(FieldLocations.RED_SOURCE, RobotContainer.swerve.getPose())));
    Logger.recordOutput("Odometry/FieldInfo/RedAmpDistance", Units.metersToInches(geoMath.distanceToPoint(FieldLocations.RED_AMP, RobotContainer.swerve.getPose())));
    Logger.recordOutput("Odometry/FieldInfo/RedSpeakerDistance", Units.metersToInches(geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, RobotContainer.swerve.getPose())));

    if (Robot.isSimulation()) {
      Logger.recordOutput("Odometry/FieldSimulation/Notes", SimulatedArena.getInstance().getGamePiecesByType("Note").toArray(new Pose3d[0]));
    }
  }

  @Override
  public void autonomousInit() {

    if(Robot.isSimulation()) {
      SimulatedArena.getInstance().resetFieldForAuto();
      RobotContainer.indexer.reset();
      RobotContainer.swerve.simReset();
    }

    RobotContainer.swerve.setBrake(false);

    _autonomousCommand = _robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (_autonomousCommand != null)
    {
      _autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    RobotContainer.shooter.autonomous(RobotContainer.swerve);
  }

  @Override
  public void teleopInit() {
    RobotContainer.swerve.setBrake(false);

    if (_autonomousCommand != null)
    {
      _autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
    
  }

  @Override
  public void teleopPeriodic() {

    boolean isRed = RobotContainer.swerve.isRedAlliance();

    if (driver.getRightTriggerAxis() > 0.5 && isRed && geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, RobotContainer.swerve.getPose()) < Units.inchesToMeters(230)) {
      aimPoint = FieldLocations.RED_SPEAKER;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = false;
    } else if(driver.getRightTriggerAxis() > 0.5 && !isRed && geoMath.distanceToPoint(FieldLocations.BLUE_SPEAKER, RobotContainer.swerve.getPose()) < Units.inchesToMeters(230)) {
      aimPoint = FieldLocations.BLUE_SPEAKER;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = false;
    } else if (driver.getRightTriggerAxis() > 0.5 && isRed && geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, RobotContainer.swerve.getPose()) > Units.inchesToMeters(230)) {
      aimPoint = FieldLocations.RED_PASS_LOCATION;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = false;
    } else if(driver.getRightTriggerAxis() > 0.5 && !isRed && geoMath.distanceToPoint(FieldLocations.BLUE_SPEAKER, RobotContainer.swerve.getPose()) > Units.inchesToMeters(230)) {
      aimPoint = FieldLocations.BLUE_PASS_LOCATION;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = false;
    } else if (driver.getRightBumperButton() && isRed) {
      aimPoint = FieldLocations.RED_AMP;
      aiming = false;
      driving = true;
      rotate = false;
      assistive = false;
    } else if (driver.getRightBumperButton() && !isRed) {
      aimPoint = FieldLocations.BLUE_AMP;
      aiming = false;
      driving = true;
      rotate = false;
      assistive = false;
    } else if (isRed && geoMath.distanceToPoint(FieldLocations.RED_SOURCE, RobotContainer.swerve.getPose()) < Units.inchesToMeters(100)) {
      aimPoint = FieldLocations.RED_SOURCE;
      aiming = false;
      driving = false;
      rotate = true;
      assistive = true;
    } else if (isRed && geoMath.distanceToPoint(FieldLocations.RED_SOURCE, RobotContainer.swerve.getPose()) > Units.inchesToMeters(180) && geoMath.distanceToPoint(FieldLocations.RED_PASS_LOCATION, RobotContainer.swerve.getPose()) > Units.inchesToMeters(225) && RobotContainer.indexer.gamePieceInShooter()) {
      aimPoint = FieldLocations.RED_PASS_LOCATION;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = true;
    }  else if (!isRed && geoMath.distanceToPoint(FieldLocations.BLUE_SOURCE, RobotContainer.swerve.getPose()) < Units.inchesToMeters(100)) {
      aimPoint = FieldLocations.BLUE_SOURCE;
      aiming = false;
      driving = false;
      rotate = true;
      assistive = true;
    } else if (!isRed && geoMath.distanceToPoint(FieldLocations.BLUE_SOURCE, RobotContainer.swerve.getPose()) > Units.inchesToMeters(180) && geoMath.distanceToPoint(FieldLocations.BLUE_PASS_LOCATION, RobotContainer.swerve.getPose()) > Units.inchesToMeters(225) && RobotContainer.indexer.gamePieceInShooter()) {
      aimPoint = FieldLocations.BLUE_PASS_LOCATION;
      aiming = true;
      driving = false;
      rotate = false;
      assistive = true;
    } else {
      aimPoint = new Pose2d();
      aiming = false;
      driving = false;
      rotate = false;
      assistive = false;
    }

    if (driving && !wasDriving) {
      CommandScheduler.getInstance().schedule(RobotContainer.swerve.driveToPose(aimPoint));
      wasDriving = true;
    } else if (!driving && wasDriving) {
      CommandScheduler.getInstance().cancelAll();
      wasDriving = false;
    } else {
      if (isRed) {
        RobotContainer.swerve.fieldRelativeTeleopSmartAim(driver.getLeftX(), driver.getLeftY(), driver.getRightX(), -2.5, aiming, rotate, assistive, aimPoint, aiming && !assistive);
      } else {
        RobotContainer.swerve.fieldRelativeTeleopSmartAim(-driver.getLeftX(), -driver.getLeftY(), driver.getRightX(), -2.5, aiming, rotate, assistive, aimPoint, aiming && !assistive);
      }
      
    }

    Pose2d speedAdjustedPose;

    if (isRed) {
      speedAdjustedPose = RobotContainer.swerve.getPose().plus(new Transform2d(new Translation2d(RobotContainer.swerve.getSwerve().getFieldVelocity().vxMetersPerSecond, RobotContainer.swerve.getSwerve().getFieldVelocity().vyMetersPerSecond), new Rotation2d(RobotContainer.swerve.getSwerve().getFieldVelocity().omegaRadiansPerSecond)).div(2.8));
      RobotContainer.shooter.SmartTeleop(geoMath.distanceToPoint(FieldLocations.RED_PASS_LOCATION, speedAdjustedPose), geoMath.distanceToPoint(FieldLocations.RED_SPEAKER, speedAdjustedPose), geoMath.distanceToPoint(FieldLocations.RED_SOURCE, RobotContainer.swerve.getPose()), RobotContainer.indexer.gamePieceInShooter(), driver.getRightTriggerAxis() > 0.5);
    } else {
      speedAdjustedPose = RobotContainer.swerve.getPose().plus(new Transform2d(new Translation2d(RobotContainer.swerve.getSwerve().getFieldVelocity().vxMetersPerSecond, RobotContainer.swerve.getSwerve().getFieldVelocity().vyMetersPerSecond), new Rotation2d(RobotContainer.swerve.getSwerve().getFieldVelocity().omegaRadiansPerSecond)).div(-2.8));
      RobotContainer.shooter.SmartTeleop(geoMath.distanceToPoint(FieldLocations.BLUE_PASS_LOCATION, speedAdjustedPose), geoMath.distanceToPoint(FieldLocations.BLUE_SPEAKER, speedAdjustedPose), geoMath.distanceToPoint(FieldLocations.BLUE_SOURCE, RobotContainer.swerve.getPose()), RobotContainer.indexer.gamePieceInShooter(), driver.getRightTriggerAxis() > 0.5);
    }

    RobotContainer.indexer.IntakeAndIndex(driver.getLeftTriggerAxis() > 0.5);

    RobotContainer.indexer.Shoot(driver.getRightTriggerAxis() > 0.8);
    
  }

  @Override
  public void disabledInit() {
    RobotContainer.swerve.setBrake(true);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    SimulatedBattery.simulationSubTick();
  }
}
