package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase{

    public final SparkMax m_winch = new SparkMax(ClimberConstants.kWinchPort, MotorType.kBrushless);
    public final SparkMaxConfig m_winchConfig = new SparkMaxConfig();

    public double winchPosition = ClimberConstants.kWinchIn;
     
    public ClimberSubsystem() {

         m_winchConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true);

        m_winchConfig.encoder
            .positionConversionFactor(1);

        m_winchConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(   ClimberConstants.kP,
                    ClimberConstants.kI,
                    ClimberConstants.kD)
            .outputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

        m_winch.configure(m_winchConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void setWinchMotor()
    {
        m_winch.getClosedLoopController().setReference(winchPosition, ControlType.kPosition);
    }

    public Command moveWinchCommand()
    {
        return new RunCommand(this::setWinchMotor, this);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Winch: ", m_winch.getEncoder().getPosition());
        SmartDashboard.putNumber("Desired Winch: ", winchPosition);
        SmartDashboard.putNumber("Winch Power: ", m_winch.get());
    }

    public Command retractWinchCommand() {
        

        return Commands.runOnce(this::setWInchRetract, this);
    }

    private void setWInchRetract()
    {
        winchPosition = ClimberConstants.kWinchIn;
    }

    public Command releaseWinchCommand() {
        

        return Commands.runOnce(this::setWinchRelease, this);
    }

    private void setWinchRelease()
    {
        winchPosition = ClimberConstants.kWinchOut;
    }
}
