// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Zone;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double robotWidth = Units.inchesToMeters(27);
    public static final double robotLength = Units.inchesToMeters(27);
    public static final double wheelRadius = Units.inchesToMeters(2);
    public static final double gearRatio = 6.12;
    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            new PIDConstants(1.5, 0.0, 0.0), // Translation PID constants 2.1
            new PIDConstants(2, 0.0, 0.0), // Rotation PID constants
            4, // Max module speed, in m/s
            Units.feetToMeters(15 / 12), // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
    );

    public static class OperatorConstants {
        public static final int driverControllerPort = 0;
        public static final int operatorControllerPort = 1;
    }

    public static class InputConstants {
        public static String triggerPressThresholdKey = "PressThreshold";
        public static double defaultTriggerPressThreshold = 0.5;
    }

    public static class SwerveConstants {
        public static final double rotationLimit = 1.75;
        public static final double maxSpeed = Units.feetToMeters(17.1);
        public static final double angularSpeed = maxSpeed / (Math.hypot(robotLength, robotWidth) / 2) / rotationLimit;
        public static final double slewRateLimit = 20;
        public static final double dtSeconds = 0.02;
    }

    public static class ShooterConstants {
        public static final int speakerShooterMotorTopID = 51;
        public static final int speakerShooterMotor2ID = 52;
        public static final int ampShooterMotor1ID = 53;
        public static final int ampShooterMotor2ID = 54;
        public static final double shooterHeight = 0;
        public static final double noteVelocity = 15;
    }

    public static class PivotConstants {
        public static final int pivotEncoderChannel = 0;
        public static final int pivotMotor1ID = 61;
        public static final int pivotMotor2ID = 62;
    }

    public static class IntakeConstants {
        public static final int intakeMotor1ID = 41;
        public static final int hangerID = 42;
        public static final int handoffMotorID = 43;
    }

    public static class LEDConstants {
        public static final int candleID1 = 36;
    }

    public static class PigeonConstants {
        public static final int pigeonID = 30;
    }

    public static class SwervePIDControllerConstants {
        public static final double driveP = 2.5;
        public static final double driveI = 0;
        public static final double driveD = 0;

        public static final double turnP = 1.5;
        public static final double turnI = 0;
        public static final double turnD = 0;
    }

    public static class LimelightConstants {
        public static final double aprilTag_Height = Units.inchesToMeters(57.5);
        public static final double LimelightHeight = 12.5;
        public static final double speakerGoalHeight = Units.inchesToMeters(83.091);
        public static final double LimelightAngle = 32;
    }

    public static class FieldConstants {
        public static final double Speaker_X_Blue = Units.inchesToMeters(-1.5 + 5);//18.033
        public static final double Speaker_X_Red = Units.inchesToMeters(652.3 - 5);
        public static final double SpeakerY = Units.inchesToMeters(218.42);
        public static final Translation2d Speaker_Red_Pose = new Translation2d(Speaker_X_Red, SpeakerY);
        public static final Translation2d Speaker_Blue_Pose = new Translation2d(Speaker_X_Blue, SpeakerY);

        public static final double Amp_X_Blue = Units.inchesToMeters(5);
        public static final double Amp_X_Red = Units.inchesToMeters(650);
        public static final double Amp_Y = Units.inchesToMeters(305);
        public static final Translation2d Amp_Blue_Pose = new Translation2d(Amp_X_Blue,Amp_Y);
        public static final Translation2d Amp_Red_Pose = new Translation2d(Amp_X_Red,Amp_Y);
        
        public static final Zone WING = new Zone(new Translation2d(0.9,7.46), new Translation2d(5.1,2.23));
    }
}
