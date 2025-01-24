package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimberSubsystem {

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
