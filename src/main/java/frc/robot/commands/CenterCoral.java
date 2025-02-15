package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClawConstants;
import frc.robot.subsystems.ClawSubsystem;

public class CenterCoral extends Command {
    private final ClawSubsystem m_clawSubsystem;
    private final boolean[] m_sensorStates;

    public CenterCoral(ClawSubsystem subsystem) {
        m_clawSubsystem = subsystem;
        addRequirements(subsystem);
        m_sensorStates = subsystem.getCoralSensorStates();
    }

    @Override
    public void initialize() {
        if (m_sensorStates[0] == true) {
            m_clawSubsystem.setLeftManipulator(-ClawConstants.kManipulatorSpeed);
        } else if (m_sensorStates[3] == true) {
            m_clawSubsystem.setRightManipulator(-ClawConstants.kManipulatorSpeed);
        }
        SmartDashboard.putBoolean("CenterCoral Running", true);
    }
    
    @Override
    public boolean isFinished() {
        if (m_sensorStates[1] == true && m_sensorStates[2] == true) {
            SmartDashboard.putBoolean("CenterCoral Running", false);
            return true;
        }
        return false;
    }
}
