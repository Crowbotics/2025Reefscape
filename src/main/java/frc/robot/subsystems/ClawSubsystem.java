package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClawSubsystem {

    private final SparkMax m_topCollector = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_bottomCollector = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_leftManipulator = new SparkMax(0, MotorType.kBrushless);
    private final SparkMax m_rightManipulator = new SparkMax(0, MotorType.kBrushless);
    
    public ClawSubsystem(){

    }

    
    public void intake() {

    }

    public void reverseIntake() {

    }

    public void centerCoral() {

    }

    public void ejectCoralLeft() {

    }

    public void ejectCoralRight() {
        
    }
    
}
