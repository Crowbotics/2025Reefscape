package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    public final SparkMax m_winch = new SparkMax(ClimberConstants.kWinchPort, MotorType.kBrushless);
     
    public ClimberSubsystem() {

    }

    private void setWinchMotor(double s)
    {
        m_winch.set(s);
    }

    public Command retractWinch() {
        return new StartEndCommand(
            () -> setWinchMotor(ClimberConstants.kWinchSpeed),
            () -> setWinchMotor(0.0),
            this);
    }

    public Command releaseWinch() {
        return new StartEndCommand(
            () -> setWinchMotor(-ClimberConstants.kWinchSpeed),
            () -> setWinchMotor(0.0),
            this);
    }

    public Command stopWinch() {
        return new RunCommand(() -> setWinchMotor(0.0), this);
    }
}
