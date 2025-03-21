// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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

    public static final double kFrontLetftOffset = -0.246826; //old: -0.361572;
    public static final double kRearLetftOffset = 0.377930;  //old: -0.087646;
    public static final double kFrontRightOffset = 0.346436; //old: -0.315186;
    public static final double kRearRightOffset = 0.215332;  //old: -0.496094;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.483;
 
    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.381;
    

    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kSlowSpeedMetersPerSecond = 4.0 /3.5;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI * 12.8;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * Math.PI * 12.8;

    //public static final int kEncoderCPR = 1024*4;
    public static final double kWheelDiameterMeters = 0.09652;
    public static final double kDriveGearRatio = 7.13;
    public static final double kDriveEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;

    public static final double kTurningGearRatio = 12.8;
    public static final double kTurningEncoderDistancePerPulse = 2*Math.PI / kTurningGearRatio;

    //PID Values for individial module TURN motors
    public static final double kPModuleTurningController = 0.275;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0.00001;
    
    //PID Values for individial module DRIVE motors
    public static final double kPModuleDriveController = 0.15;
    public static final double kIModuleDriveController = 0; //0.0001;
    public static final double kDModuleDriveController = 0.001;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAuxControllerPort = 1;
  }


  public static final class ArmConstants {
    public static final int kArmPort1 = 9;
    public static final int kArmPort2 = 10;

    public static final double kGroundFront = 0.1825;
    public static final double kL1Front     = 0.31;
    public static final double kL2Front     = 0.38;
    public static final double kStraightUp  = 0.50;
    public static final double kL2Rear      = 0.62;
    public static final double kL1Rear      = 0.675;
    public static final double kGroundRear  = 0.8400;

    public static final double kAlgaeFront  = 0.34;
    public static final double kAlgaeBack   = 0.76;

    public static final double kBeginClimb  = 0.35;
    public static final double kEndClimb    = 0.25;

    public static final double kP = 10;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kMaxOutput = 0.5;
    public static final double kMinOutput = -0.5;

    //Feedforward Constants
    public static final double kS = 0.05;
    public static final double kG = 0.05;
    public static final double kV = 0.05;
  }

  public static final class ClawConstants {
    public static final int kTopPort = 11;
    public static final int kBottomPort = 12;
    public static final int kLeftPort = 13;
    public static final int kRightPort = 14;
    public static final int kKickerPort = 15;

    public static final int kCoralSensor0 = 0;
    public static final int kCoralSensor1 = 1;
    public static final int kCoralSensor2 = 2;
    public static final int kCoralSensor3 = 3;

    public static final double kCollectorSpeed = 1;
    public static final double kReverseCollectorSpeed = -0.20;
    public static final double kAlgaeRemovalSpeed = -0.6;

    public static final double kManipulatorSpeed = 0.3; // 0 - 1
    public static final double kCenteringSpeed = 0.8;

    public static final double kPuncherOutSpeed = 0.6;
    public static final double kPuncherRetractSpeed = -0.8;

    public static final double kAutoIntakeDuration = 0.75;
    public static final double kAutoOutakeDuration = 0.75;
  }

  public static final class ClimberConstants {
    public static final int kWinchPort = 16;

    public static final double kWinchSpeed = 0.6;

    public static final double kWinchOut = -120.0;
    public static final double kWinchIn = -0.0;

    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
  }
}
