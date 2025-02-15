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
import frc.robot.commands.CenterCoral;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ClawSubsystem m_claw = new ClawSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  Joystick m_auxController = new Joystick(OIConstants.kAuxControllerPort);

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

    m_arm.setDefaultCommand(new RunCommand(m_arm::setArmPosition, m_arm));

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
    JoystickButton driverA = new JoystickButton(m_driverController, 1);
    JoystickButton driverB = new JoystickButton(m_driverController, 2);
    JoystickButton driverX = new JoystickButton(m_driverController, 3);
    JoystickButton driverY = new JoystickButton(m_driverController, 4);
    JoystickButton driverLB = new JoystickButton(m_driverController, 5);
    JoystickButton driverRB = new JoystickButton(m_driverController,6);
    JoystickButton driverSelect = new JoystickButton(m_driverController,7);
    JoystickButton driverStart = new JoystickButton(m_driverController, 8);
    JoystickButton driverLS = new JoystickButton(m_driverController,9);
    JoystickButton driverRS = new JoystickButton(m_driverController,10);

    JoystickButton auxA = new JoystickButton(m_auxController, 1);
    JoystickButton auxB = new JoystickButton(m_auxController, 2);
    JoystickButton auxX = new JoystickButton(m_auxController, 3);
    JoystickButton auxY = new JoystickButton(m_auxController, 4);
    JoystickButton auxLB = new JoystickButton(m_auxController, 5);
    JoystickButton auxRB = new JoystickButton(m_auxController,6);
    JoystickButton auxSelect = new JoystickButton(m_auxController,7);
    JoystickButton auxStart = new JoystickButton(m_auxController, 8);
    JoystickButton auxLS = new JoystickButton(m_auxController,9);
    JoystickButton auxRS = new JoystickButton(m_auxController,10);

    new Trigger(driverA.onTrue(Commands.runOnce(m_robotDrive::zeroHeading , m_robotDrive)));
    new Trigger(driverB.onTrue(Commands.runOnce(m_robotDrive::resetPose, m_robotDrive)));

    // Center coral
    new Trigger(driverX.onTrue(
      m_claw.outtakeExtraCoral()
      .andThen(m_claw.centerCoral())
    ));

    new Trigger(driverLB.onTrue(Commands.runOnce(m_arm::moveArmForward, m_arm)));
    new Trigger(driverRB.onTrue(Commands.runOnce(m_arm::moveArmBackward, m_arm)));
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



