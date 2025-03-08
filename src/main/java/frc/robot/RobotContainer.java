// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();
  private final ClimberSubsystem m_climber = new ClimberSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  SlewRateLimiter slewX = new SlewRateLimiter(10);
  SlewRateLimiter slewY = new SlewRateLimiter(10);
  SlewRateLimiter slewR = new SlewRateLimiter(8);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Register commands with PathPlanner
    NamedCommands.registerCommand("moveArm", m_arm.moveArmCommand());
    NamedCommands.registerCommand("moveArmStateUp", m_arm.moveArmUpCommand());
    NamedCommands.registerCommand("moveArmStateDown", m_arm.moveArmDownCommand());
    NamedCommands.registerCommand("intake", m_claw.intakeCommand());
    NamedCommands.registerCommand("stopIntake", m_claw.stopIntakeCommand());
    NamedCommands.registerCommand("outake", m_claw.reverseIntakeCommand().andThen(m_claw.retractPuncherCommand()));
    NamedCommands.registerCommand("stopOutake", m_claw.retractPuncherCommand());
    m_chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_chooser);
    // Configure default commands
    m_arm.setDefaultCommand(m_arm.moveArmCommand());
    m_climber.setDefaultCommand(m_climber.moveWinchCommand());

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    slewX.calculate(-MathUtil.applyDeadband(m_driverController.getRawAxis(1), 0.075) 
                        * DriveConstants.kMaxSpeedMetersPerSecond),
                    slewY.calculate(-MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.075) 
                        * DriveConstants.kMaxSpeedMetersPerSecond),
                    -MathUtil.applyDeadband(m_driverController.getRawAxis(4), 0.075)
                        * DriveConstants.kMaxSpeedMetersPerSecond*1.6,
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

  private void configureButtonBindings() 
  {
    JoystickButton driverA = new JoystickButton(m_driverController, 1);
    JoystickButton driverB = new JoystickButton(m_driverController, 2);
    JoystickButton driverX = new JoystickButton(m_driverController, 3);
    JoystickButton driverY = new JoystickButton(m_driverController, 4);
    JoystickButton driverLeftBumper = new JoystickButton(m_driverController, 5);
    JoystickButton driverRightBumper = new JoystickButton(m_driverController,6);
    JoystickButton driverSelect = new JoystickButton(m_driverController,7);
    JoystickButton driverStart = new JoystickButton(m_driverController, 8);
    //JoystickButton driverLeftStickIn = new JoystickButton(m_driverController,9);
    //JoystickButton driverRightStickIn = new JoystickButton(m_driverController,10);
    Trigger driverLeftTrigger = new Trigger(() -> m_driverController.getRawAxis(2) > 0.5);
    Trigger driverRightTrigger = new Trigger(() -> m_driverController.getRawAxis(3) > 0.5);
    POVButton driverPOVU = new POVButton(m_driverController, 0);
    POVButton driverPOVR = new POVButton(m_driverController, 90);
    //POVButton driverPOVD = new POVButton(m_driverController, 180);
    POVButton driverPOVL = new POVButton(m_driverController, 270);

    //Claw Buttons
    new Trigger(driverRightTrigger.whileTrue(m_claw.intakeCommand()));
    new Trigger(driverLeftTrigger.whileTrue(m_claw.reverseIntakeCommand()).onFalse(m_claw.retractPuncherCommand()));
    //new Trigger(driverPOVR.whileTrue(m_claw.reverseIntakeCommand()).onFalse(m_claw.retractPuncherCommand()));
    new Trigger(driverPOVL.whileTrue(m_claw.scoreLeftCommand()));
    new Trigger(driverPOVU.whileTrue(m_claw.scoreRightCommand()));

    //Arm Buttons
    new Trigger(driverRightBumper.onTrue(m_arm.moveArmUpCommand()));
    new Trigger(driverLeftBumper.onTrue(m_arm.moveArmDownCommand()));
    new Trigger(driverY.onTrue(m_arm.moveArmStraightUpCommand()));

    //Climber Buttons
    new Trigger(driverSelect.onTrue(
      new SequentialCommandGroup(
        m_arm.moveArmBeginClimbCommand(),
        m_arm.moveArmCommand().until(m_arm::isArmAtSetpoint),
        m_climber.releaseWinchCommand())));

    new Trigger(driverStart.onTrue(
      new SequentialCommandGroup(
        m_arm.moveArmEndClimbCommand(),
        m_arm.moveArmCommand().until(m_arm::isArmAtSetpoint), 
        m_climber.retractWinchCommand())));

    //Drivetrain Buttons
    new Trigger(driverA.onTrue(m_robotDrive.zeroHeading()));
    new Trigger(driverB.onTrue(m_robotDrive.zeroPose()));
    new Trigger(driverX.whileTrue(
      new RunCommand(
        () ->
            m_robotDrive.drive(
                // Multiply by max speed to map the joystick unitless inputs to actual units.
                // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                // converting them to actual units.
                slewX.calculate(-MathUtil.applyDeadband(m_driverController.getRawAxis(1), 0.075) 
                    * DriveConstants.kMaxSpeedMetersPerSecond/3.0),
                slewY.calculate(-MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.075) 
                    * DriveConstants.kMaxSpeedMetersPerSecond/3.0),
                -MathUtil.applyDeadband(m_driverController.getRawAxis(4), 0.075)
                    * DriveConstants.kMaxSpeedMetersPerSecond,
                true),
        m_robotDrive))
    );

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



