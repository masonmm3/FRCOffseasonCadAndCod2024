// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Swerve;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import frc.robot.Util.Constants;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/** Add your docs here. */
public class SwerveHardware implements SwerveIO{
    private double _maximumSpeed = Units.feetToMeters(20.1);
    private File _swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");;
    private SwerveDrive _swerveDrive;
    private SwerveModule[] _modules;  
    private SwerveMotor _lfDriveMotor;
    private SwerveMotor _lfAngleMotor;
    private SwerveMotor _rfDriveMotor;
    private SwerveMotor _rfAngleMotor;
    private SwerveMotor _lrDriveMotor;
    private SwerveMotor _lrAngleMotor;
    private SwerveMotor _rrDriveMotor;
    private SwerveMotor _rrAngleMotor;

    public SwerveHardware() 
    {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.NONE;
        try 
        {

            _swerveDrive = new SwerveParser(_swerveJsonDirectory).createSwerveDrive(_maximumSpeed); 

        } catch (Exception e) {

            throw new RuntimeException(e);

        }

        _swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle.
        _swerveDrive.setCosineCompensator(false);//!SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
        _swerveDrive.setAngularVelocityCompensation(true, true, 0.1); //Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
        _swerveDrive.setModuleEncoderAutoSynchronize(true, 1); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
        _swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

        _modules = _swerveDrive.getModules();

        _lfAngleMotor = _modules[0].getAngleMotor();
        _lfDriveMotor = _modules[0].getDriveMotor();

        _rfAngleMotor = _modules[1].getAngleMotor();
        _rfDriveMotor = _modules[1].getDriveMotor();

        _lrAngleMotor = _modules[2].getAngleMotor();
        _lrDriveMotor = _modules[2].getDriveMotor();

        _rrAngleMotor = _modules[3].getAngleMotor();
        _rrDriveMotor = _modules[3].getDriveMotor();
    }

    @Override
    public void driveTeleop(double translationX, double translationY, double angularRotationX, boolean fieldRelative) {
        _swerveDrive.drive(
            SwerveMath.scaleTranslation(
                new Translation2d(
                    translationX * _swerveDrive.getMaximumChassisVelocity(),
                    translationY * _swerveDrive.getMaximumChassisVelocity()),
                     0.8),
            Math.pow(angularRotationX, 3) * _swerveDrive.getMaximumChassisAngularVelocity(),
            fieldRelative,
            false);
    }

    @Override
    public void drive(Translation2d translation, double rotation, boolean fieldRelative)
    {
        _swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    public void drivePathPlanner(ChassisSpeeds speeds, SwerveModuleState[] modules, Force[] forwardForce) {
        _swerveDrive.drive(speeds, modules, forwardForce);
    }

    @Override
    public void driveFieldOriented(ChassisSpeeds velocity) {
        _swerveDrive.driveFieldOriented(velocity);
    }
    
    @Override
    public void driveChasisSpeeds(ChassisSpeeds velocity)
    {
        _swerveDrive.drive(velocity);
    }

    @Override
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds)
    {
        _swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        _swerveDrive.setMotorIdleMode(enabled);
    }

    

    @Override
    public void zeroGyro()
    {
        _swerveDrive.zeroGyro();
    }

    @Override
    public void resetOdometry(Pose2d initialHolonomicPose)
    {
        _swerveDrive.resetOdometry(initialHolonomicPose);
    }

    @Override
    public boolean isRedAlliance()
    {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    @Override
    public void zeroGyroWithAlliance()
    {
      if (isRedAlliance())
      {
        zeroGyro();
        //Set the pose 180 degrees
        resetOdometry(
            new Pose2d(
                getPose().getTranslation(),
                Rotation2d.fromDegrees(180)
            )
        );
      } else
      {
        zeroGyro();
      }
    }

    @Override
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle)
    {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return _swerveDrive
                .swerveController
                    .getTargetSpeeds(
                        scaledInputs.getX(),
                        scaledInputs.getY(),
                        angle.getRadians(),
                        getHeading().getRadians(),
                        Constants.MAX_SPEED
                    );
    }

    
    @Override
    public SwerveDriveConfiguration getSwerveDriveConfiguration()
    {
        return _swerveDrive.swerveDriveConfiguration;
    }

    @Override
    public ChassisSpeeds getFieldVelocity()
    {
        return _swerveDrive.getFieldVelocity();
    }

    @Override 
    public ChassisSpeeds getRobotVelocity() {
        return _swerveDrive.getRobotVelocity();
    }

    @Override
    public Pose2d getPose()
    {
            return _swerveDrive.getPose();
    }

    @Override
    public Pose2d getSimPose() {
        if (Robot.isReal()) {
            return null;
        } else {
            return _swerveDrive.getSimulationDriveTrainPose().get();
        }
    }

    @Override
    public Rotation2d getHeading()
    {
        return getPose().getRotation();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return _swerveDrive.kinematics;
    }

    @Override
    public SwerveModuleState[] getModuleState() {
        return _swerveDrive.getStates();
    }

    @Override
    public SwerveModuleState[] getSetpoint(ChassisSpeeds speeds) {
        return _swerveDrive.toServeModuleStates(speeds, true);
    }

    @Override
    public Field2d getField() {
        return _swerveDrive.field;
    }

    @Override
    public SwerveDrive getSwerve() {
        return _swerveDrive;
    }

    @Override
    public void updateOdometry() {
        _swerveDrive.updateOdometry();
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {

        //left front
        inputs.LeftFrontModuleAngleCurrent = _lfAngleMotor.getAppliedOutput();
        inputs.LeftFrontModuleAngleEncoderAngle = _modules[0].getAbsolutePosition();
        inputs.LeftFrontModuleAngleInternalAngle = _lfAngleMotor.getPosition();
        inputs.LeftFrontModuleAngleVelocity = _lfAngleMotor.getVelocity();
        inputs.LeftFrontModuleAngleVolt = _lfAngleMotor.getVoltage();
        
        inputs.LeftFrontModuleDriveCurrent = _lfDriveMotor.getAppliedOutput();
        inputs.LeftFrontModuleDrivePosition = _lfDriveMotor.getPosition();
        inputs.LeftFrontModuleDriveVelocity = _lfDriveMotor.getVelocity();
        inputs.LeftFrontModuleDriveVolt = _lfDriveMotor.getVoltage();


        //right front
        inputs.RightFrontModuleAngleCurrent = _rfAngleMotor.getAppliedOutput();
        inputs.RightFrontModuleAngleEncoderAngle = _modules[1].getAbsolutePosition();
        inputs.RightFrontModuleAngleInternalAngle = _rfAngleMotor.getPosition();
        inputs.RightFrontModuleAngleVelocity = _rfAngleMotor.getVelocity();
        inputs.RightFrontModuleAngleVolt = _rfAngleMotor.getVoltage();
        
        inputs.RightFrontModuleDriveCurrent = _rfDriveMotor.getAppliedOutput();
        inputs.RightFrontModuleDrivePosition = _rfDriveMotor.getPosition();
        inputs.RightFrontModuleDriveVelocity = _rfDriveMotor.getVelocity();
        inputs.RightFrontModuleDriveVolt = _rfDriveMotor.getVoltage();

        //left rear
        inputs.LeftRearModuleAngleCurrent = _lrAngleMotor.getAppliedOutput();
        inputs.LeftRearModuleAngleEncoderAngle = _modules[2].getAbsolutePosition();
        inputs.LeftRearModuleAngleInternalAngle = _lrAngleMotor.getPosition();
        inputs.LeftRearModuleAngleVelocity = _lrAngleMotor.getVelocity();
        inputs.LeftRearModuleAngleVolt = _lrAngleMotor.getVoltage();
        
        inputs.LeftRearModuleDriveCurrent = _lrDriveMotor.getAppliedOutput();
        inputs.LeftRearModuleDrivePosition = _lrDriveMotor.getPosition();
        inputs.LeftRearModuleDriveVelocity = _lrDriveMotor.getVelocity();
        inputs.LeftRearModuleDriveVolt = _lrDriveMotor.getVoltage();


        //right rear
        inputs.RightRearModuleAngleCurrent = _rrAngleMotor.getAppliedOutput();
        inputs.RightRearModuleAngleEncoderAngle = _modules[3].getAbsolutePosition();
        inputs.RightRearModuleAngleInternalAngle = _rrAngleMotor.getPosition();
        inputs.RightRearModuleAngleVelocity = _rrAngleMotor.getVelocity();
        inputs.RightRearModuleAngleVolt = _rrAngleMotor.getVoltage();
        
        inputs.RightRearModuleDriveCurrent = _rrDriveMotor.getAppliedOutput();
        inputs.RightRearModuleDrivePosition = _rrDriveMotor.getPosition();
        inputs.RightRearModuleDriveVelocity = _rrDriveMotor.getVelocity();
        inputs.RightRearModuleDriveVolt = _rrDriveMotor.getVoltage();
    }

}
