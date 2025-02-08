package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase{

    public final SparkMax m_winch = new SparkMax(0, MotorType.kBrushless);
     
    public ClimberSubsystem() {

    }

    public void windWinch() {

    }

    public void releaseWinch() {

    }

    public void stopWinch() {
        
    }
}
