// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // A SendableChooser to pick autos
    private SendableChooser<Command> m_chooser = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(1), 0.075) 
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.075) 
                        * DriveConstants.kMaxSpeedMetersPerSecond,
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(4), 0.075)
                        * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    true),
            m_robotDrive));

                // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */

   private double kSlowmoMultiplier = 1;

   public void slowmo () { kSlowmoMultiplier = 0.4;}

   


  private void configureButtonBindings() 
  {
    JoystickButton aButton = new JoystickButton(m_driverController, 1);
    new Trigger(aButton.onTrue(new RunCommand(m_robotDrive::zeroHeading , m_robotDrive)).onFalse(m_robotDrive.getDefaultCommand()));

    JoystickButton bButton = new JoystickButton(m_driverController, 2);
    new Trigger(bButton.onTrue(new RunCommand(m_robotDrive::resetPose, m_robotDrive)).onFalse(m_robotDrive.getDefaultCommand()));
/* 
    JoystickButton rbButton = new JoystickButton(m_driverController, 5);
    new trigger (rbButton.onTrue( ()))*/
  }



  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return m_chooser.getSelected();
    
  }
}

/**
 * 
 * // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            Pose2d.kZero,
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, Rotation2d.kZero),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
 * 
 * 
 */


