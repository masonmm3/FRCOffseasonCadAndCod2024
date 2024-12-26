// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import org.littletonrobotics.junction.AutoLog;


/** Add your docs here. */
public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double LeftFlywheelSpeed;
        public double LeftFlywheelInputSpeed;
        public double RightFlywheelSpeed;
        public double RightFlywheelInputSpeed;

        public double ArmAngle;
        public double ArmSetpoint;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void IdleSpin() {}

    public default void AimAtSpeaker(double distanceToSpeaker) {}

    public default void AimAtPass(double distanceToPass) {}

    public default void StowShooter() {}

    public default void FlywheelSpeedUpSpeaker(double distanceToSpeaker) {}

    public default void FlywheelSpeedUpPass(double distanceToPass) {}

    public default void reset() {}

    public default double getShooterAngle() {
        return 15;
    }

    public default double getShooterError() {
        return 0;
    }

    public default double getArmError() {
        return 0;
    }

    public default double gettShooterSpeeds() {
        return 0;
    }
}
