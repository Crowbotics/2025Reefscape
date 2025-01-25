// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {

    public static final String message = "Lofty's variables are the best!";

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 3;
    public static final int kFrontRightDriveMotorPort = 5;
    public static final int kRearRightDriveMotorPort = 7;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 4;
    public static final int kFrontRightTurningMotorPort = 6;
    public static final int kRearRightTurningMotorPort = 8;

    public static final int kFrontLeftTurningEncoderPort = 22;
    public static final int kRearLeftTurningEncoderPort = 24;
    public static final int kFrontRightTurningEncoderPort = 26;
    public static final int kRearRightTurningEncoderPort = 28;

    public static final double kFrontLetftOffset = -0.361572;
    public static final double kRearLetftOffset = -0.087646;
    public static final double kFrontRightOffset = -0.315186;
    public static final double kRearRightOffset = -0.496094;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;
    public static final double kTrackWidth = 0.548;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.548;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 10;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI;

    public static final int kEncoderCPR = 1024*4;
    public static final double kWheelDiameterMeters = 0.0827;
    public static final double kDriveGearRatio = 5.96;
    public static final double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    public static final double kTurningGearRatio = 12.8;
    public static final double kTurningEncoderDistancePerPulse = 2*Math.PI / kTurningGearRatio;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
  }

  public static final class PIDConstants {

    //PID Values for individial module TURN motors
    public static final double kPModuleTurningController = 0.45;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0.001;

    //PID Values for individial module DRIVE motors
    public static final double kPModuleDriveController = .1;
    public static final double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
