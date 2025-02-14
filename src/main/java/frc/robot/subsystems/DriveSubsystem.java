// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningEncoderPort,
          DriveConstants.kFrontLetftOffset);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningEncoderPort,
          DriveConstants.kRearLetftOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningEncoderPort,
          DriveConstants.kFrontRightOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningEncoderPort,
          DriveConstants.kRearRightOffset);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  double dp = ModuleConstants.kPModuleDriveController;
  double di = ModuleConstants.kIModuleDriveController;
  double dd = ModuleConstants.kDModuleDriveController;

  double tp = ModuleConstants.kPModuleTurningController;
  double ti = ModuleConstants.kIModuleTurningController;
  double td = ModuleConstants.kDModuleTurningController;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    RobotConfig ppconfig;
    try{
      ppconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
      ppconfig = null;
    }

    SmartDashboard.putNumber("Drive P: ", dp);
    SmartDashboard.putNumber("Drive I: ", di);
    SmartDashboard.putNumber("Drive D: ", dd);

    SmartDashboard.putNumber("Turn P: ", tp);
    SmartDashboard.putNumber("Turn I: ", ti);
    SmartDashboard.putNumber("Turn D: ", td);

    AutoBuilder.configure(this::getPose, this::resetOdometry, this::getRobotRelativeSpeeds, 
    (speeds, ff)->drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false), 
    
    new PPHolonomicDriveController(
      new PIDConstants(15, 1, 0), new PIDConstants(5)), ppconfig, 
      ()->{
        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }, this);

    
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
   return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(
    new SwerveModuleState[] {m_frontLeft.getState(),
    m_frontRight.getState(),
    m_rearLeft.getState(),
    m_rearRight.getState()});
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

        //SmartDashboard.putNumber("FL Angle: ", m_frontLeft.getCancoderAngleInDegrees());
        //SmartDashboard.putNumber("FL Distance: ", m_frontLeft.getDriveDistance());
        //SmartDashboard.putNumber("FR Angle: ", m_frontRight.getCancoderAngleInDegrees());
        //SmartDashboard.putNumber("FR Distance: ", m_frontRight.getDriveDistance());
        //SmartDashboard.putNumber("RL Angle: ", m_rearLeft.getCancoderAngleInDegrees());
        //SmartDashboard.putNumber("RL Distance: ", m_rearLeft.getDriveDistance());
        //SmartDashboard.putNumber("RR Angle: ", m_rearRight.getCancoderAngleInDegrees());
        //SmartDashboard.putNumber("RR Distance: ", m_rearRight.getDriveDistance());

        SmartDashboard.putNumber("Actual Velocity: ", m_frontLeft.getSpeed());
        SmartDashboard.putNumber("Desired Velocity: ", m_frontLeft.getDesiredSpeed());
        SmartDashboard.putNumber("Drive Output: ", m_frontLeft.getDriveOutput());

        double temp_dp = SmartDashboard.getNumber("Drive P: ", 0.0);
        double temp_di = SmartDashboard.getNumber("Drive I: ", 0.0);
        double temp_dd = SmartDashboard.getNumber("Drive D: ", 0.0);

        double temp_tp = SmartDashboard.getNumber("Turn P: ", 0.0);
        double temp_ti = SmartDashboard.getNumber("Turn I: ", 0.0);
        double temp_td = SmartDashboard.getNumber("Turn D: ", 0.0);

        if(temp_dp != dp || temp_di != di || temp_dd != dd || temp_tp != tp || temp_ti != ti || temp_td!= td)
        {
          //m_frontLeft.setPID(temp_dp, temp_di, temp_dd, temp_tp, temp_ti, temp_td);
          //m_frontRight.setPID(temp_dp, temp_di, temp_dd, temp_tp, temp_ti, temp_td);
          //m_rearLeft.setPID(temp_dp, temp_di, temp_dd, temp_tp, temp_ti, temp_td);
          //m_rearRight.setPID(temp_dp, temp_di, temp_dd, temp_tp, temp_ti, temp_td);

          dp = temp_dp; di = temp_di; dd = temp_dd; tp = temp_tp; ti = temp_ti; td = temp_td;
        }


        //SmartDashboard.putString("PID String: ", m_frontLeft.getPID());




        SmartDashboard.putNumber("Gyro: ", m_gyro.getAngle());
        SmartDashboard.putNumber("X: ", getPose().getX());
        SmartDashboard.putNumber("Y: ", getPose().getY());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetPose()
  {
    resetOdometry(new Pose2d());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    //xSpeed = -4;

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
