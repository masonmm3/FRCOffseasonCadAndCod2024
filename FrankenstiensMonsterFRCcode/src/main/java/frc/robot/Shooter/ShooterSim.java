// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Util.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterSim implements ShooterIO{
    private FlywheelSim _leftShooters;
    private FlywheelSim _rightShooters;
    private SingleJointedArmSim _armMotor;

    private static InterpolatingDoubleTreeMap _aimAtSpeaker = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap _aimAtPass = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap _speakerRPM = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap _passRPM = new InterpolatingDoubleTreeMap();

    private PIDController _leftPID = new PIDController(0.04, 0.00, 0.0000);
    private PIDController _rightPID = new PIDController(0.04, 0.00, 0.0000);
    private SimpleMotorFeedforward _leftFeed = new SimpleMotorFeedforward(0.0010, 0.002);
    private SimpleMotorFeedforward _rightFeed = new SimpleMotorFeedforward(0.0010, 0.002);

    private PIDController anglePID = new PIDController(0.2, 0, 0.0100);
    private ArmFeedforward angleFeedForward = new ArmFeedforward(0, 0.0405, 2);

    public ShooterSim() {

        anglePID.enableContinuousInput(-180, 180);

        _armMotor = new SingleJointedArmSim(DCMotor.getNEO(1), (225/29)*15,SingleJointedArmSim.estimateMOI(Units.inchesToMeters(14), Units.lbsToKilograms(16.898)),Units.inchesToMeters(14), Units.degreesToRadians(15), Units.degreesToRadians(75), true, Units.degreesToRadians(15));
        _leftShooters = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1), DCMotor.getNEO(1), 0.00064);
        _rightShooters = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 1), DCMotor.getNEO(1), 0.00064);

        _aimAtSpeaker.put(1.25, 74.0);
        _aimAtSpeaker.put(1.84, 62.0);
        _aimAtSpeaker.put(3.0, 48.0);
        _aimAtSpeaker.put(4.0, 41.0);
        _aimAtSpeaker.put(4.5, 37.0);
        _aimAtSpeaker.put(5.0, 34.0);
        _aimAtSpeaker.put(5.5, 33.0);
        _aimAtSpeaker.put(6.5, 23.0);


        _aimAtPass.put(10.2, 65.0);
        _aimAtPass.put(11.0, 73.0);

        _speakerRPM.put(1.25, 2500.0);
        _speakerRPM.put(6.0, 5400.0);

        _passRPM.put(8.0, 1500.0);
        _passRPM.put(10.2, 1780.0);
        _passRPM.put(11.0, 2000.0);

    }

    @Override
    public void IdleSpin() {

        double leftVoltage = MathUtil.clamp(_leftFeed.calculate(ShooterConstants.IDLE_RPM) + _leftPID.calculate(_leftShooters.getAngularVelocityRPM(), ShooterConstants.IDLE_RPM), -12, 12);
        double rightVoltage = MathUtil.clamp(_rightFeed.calculate(ShooterConstants.IDLE_RPM) + _rightPID.calculate(_rightShooters.getAngularVelocityRPM(), ShooterConstants.IDLE_RPM), -12, 12);

        _leftShooters.setInputVoltage(leftVoltage);
        _rightShooters.setInputVoltage(rightVoltage);
    }

    @Override
    public void AimAtSpeaker(double distanceToSpeaker) {

        Rotation2d target = new Rotation2d(Units.degreesToRadians(_aimAtSpeaker.get(distanceToSpeaker)));

        double angleVoltage = MathUtil.clamp(angleFeedForward.calculate(Angle.ofBaseUnits(target.getDegrees(), edu.wpi.first.units.Units.Degree), AngularVelocity.ofBaseUnits(target.getDegrees()-Units.radiansToDegrees(_armMotor.getAngleRads()), edu.wpi.first.units.Units.DegreesPerSecond)).baseUnitMagnitude() + anglePID.calculate(Units.radiansToDegrees(_armMotor.getAngleRads()), target.getDegrees()), -12, 12);
    
        _armMotor.setInputVoltage(angleVoltage);
    }

    @Override
    public void AimAtPass(double distanceToPass) {
        Rotation2d target = new Rotation2d(Units.degreesToRadians(_aimAtPass.get(distanceToPass)));

        double angleVoltage = MathUtil.clamp(angleFeedForward.calculate(Angle.ofBaseUnits(target.getDegrees(), edu.wpi.first.units.Units.Degree), AngularVelocity.ofBaseUnits(target.getDegrees()-Units.radiansToDegrees(_armMotor.getAngleRads()), edu.wpi.first.units.Units.DegreesPerSecond)).baseUnitMagnitude() + anglePID.calculate(Units.radiansToDegrees(_armMotor.getAngleRads()),target.getDegrees()), -12, 12);
    
        _armMotor.setInputVoltage(angleVoltage);
    }

    @Override
    public void StowShooter() {
        Rotation2d target = new Rotation2d(Units.degreesToRadians(15));

        double angleVoltage = MathUtil.clamp(angleFeedForward.calculate(Angle.ofBaseUnits(target.getDegrees(), edu.wpi.first.units.Units.Degree), AngularVelocity.ofBaseUnits(target.getDegrees()-Units.radiansToDegrees(_armMotor.getAngleRads()), edu.wpi.first.units.Units.DegreesPerSecond)).baseUnitMagnitude() + anglePID.calculate(Units.radiansToDegrees(_armMotor.getAngleRads()), target.getDegrees()), -12, 12);
    
        _armMotor.setInputVoltage(angleVoltage);
    }

    @Override
    public void FlywheelSpeedUpSpeaker(double distanceToSpeaker) {

        double target = _speakerRPM.get(distanceToSpeaker);

        double leftVoltage = MathUtil.clamp(_leftFeed.calculate(target-20) + _leftPID.calculate(_leftShooters.getAngularVelocityRPM(), target-20), -12, 12);
        double rightVoltage = MathUtil.clamp(_rightFeed.calculate(target) + _rightPID.calculate(_rightShooters.getAngularVelocityRPM(), target), -12, 12);

        _leftShooters.setInputVoltage(leftVoltage);
        _rightShooters.setInputVoltage(rightVoltage);
    }

    @Override
    public void FlywheelSpeedUpPass(double distanceToPass) {

        double target = _passRPM.get(distanceToPass);

        double leftVoltage = MathUtil.clamp(_leftFeed.calculate(target-20) + _leftPID.calculate(_leftShooters.getAngularVelocityRPM(), target-20), -12, 12);
        double rightVoltage = MathUtil.clamp(_rightFeed.calculate(target-20) + _rightPID.calculate(_rightShooters.getAngularVelocityRPM(), target-20), -12, 12);

        _leftShooters.setInputVoltage(leftVoltage);
        _rightShooters.setInputVoltage(rightVoltage);
    }

    @Override
    public double getShooterAngle() {
        return new Rotation2d(_armMotor.getAngleRads() - Units.degreesToRadians(15)).getDegrees();
    }

    @Override
    public double getArmError() {
        return Math.abs(anglePID.getError());
    }

    @Override
    public double getShooterError() {
        return (Math.abs(_leftPID.getError())+Math.abs(_rightPID.getError()))/2;
    }

    public double gettShooterSpeeds() {
        return (_leftShooters.getAngularVelocityRPM() + _rightShooters.getAngularVelocityRPM())/2;
    }

    @Override
    public void reset() {
        _leftShooters.setAngularVelocity(0);
        _rightShooters.setAngularVelocity(0);
        _armMotor.setState(Units.degreesToRadians(15), 0);
    }


    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        _leftShooters.update(0.02);
        _rightShooters.update(0.02);
        _armMotor.update(0.02);

        inputs.ArmAngle = Units.radiansToDegrees(_armMotor.getAngleRads());
        inputs.ArmSetpoint = anglePID.getError()+Units.radiansToDegrees(_armMotor.getAngleRads());

        inputs.LeftFlywheelSpeed = _leftShooters.getAngularVelocityRPM();
        inputs.LeftFlywheelInputSpeed = _leftPID.getError() + _leftShooters.getAngularVelocityRPM();

        inputs.RightFlywheelSpeed = _rightShooters.getAngularVelocityRPM();
        inputs.RightFlywheelInputSpeed = _rightPID.getError() + _rightShooters.getAngularVelocityRPM();
    }
}
