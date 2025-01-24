// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PIDConstants;

public class SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final SparkMaxConfig m_driveConfig;
  private final SparkMaxConfig m_turnConfig;


 
  private final CANcoder m_turningEncoder;

  private final PIDController m_drivePIDController =
      new PIDController(PIDConstants.kPModuleDriveController,
                        PIDConstants.kIModuleDriveController,
                        PIDConstants.kDModuleDriveController);

  // Using a TrapezoidProfile PIDController to allow for smooth turning
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          PIDConstants.kPModuleTurningController,
          PIDConstants.kIModuleTurningController,
          PIDConstants.kDModuleTurningController,
          new TrapezoidProfile.Constraints(
              ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
              ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double offset) {
    m_driveMotor = new SparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorChannel, MotorType.kBrushless);
    m_driveConfig = new SparkMaxConfig();
    m_turnConfig = new SparkMaxConfig();

    m_driveConfig .inverted(true)
                  .idleMode(IdleMode.kBrake);
    m_driveConfig.encoder .positionConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse)
                          .velocityConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse / 60.0);

    m_turnConfig  .inverted(true);

    m_driveMotor.configure(m_driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(m_turnConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_turningEncoder = new CANcoder(turningEncoderChannel);
    CANcoderConfiguration toApply = new CANcoderConfiguration();
    toApply.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
    toApply.MagnetSensor.MagnetOffset = offset;
    m_turningEncoder.getConfigurator().apply(toApply);

    //m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveMotor.getEncoder().getVelocity(), new Rotation2d(getCancoderAngleInRadians()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveMotor.getEncoder().getPosition(), new Rotation2d(getCancoderAngleInRadians()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getCancoderAngleInRadians());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity(), desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(
          getCancoderAngleInRadians(), desiredState.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    m_driveMotor.set(driveOutput);
    m_turningMotor.set(turnOutput);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveMotor.getEncoder().setPosition(0);
    m_turningEncoder.setPosition(0);
  }

  public double getDriveDistance()
  {
    return m_driveMotor.getEncoder().getPosition();
  }

  public double getCancoderAngleInRadians()
  {
    return Units.rotationsToRadians(m_turningEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getCancoderAngleInDegrees()
  {
    return Units.rotationsToDegrees(m_turningEncoder.getAbsolutePosition().getValueAsDouble());
  }
}
