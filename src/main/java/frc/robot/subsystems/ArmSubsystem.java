package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ArmSubsystem {

    public final SparkMax m_pivotMotor1 = new SparkMax(0, MotorType.kBrushless);
    public final SparkMax m_pivotMotor2 = new SparkMax(0, MotorType.kBrushless);

    
    public ArmSubsystem() {

    }

    public void moveArmForward() {

    }

    public void moveArmBackward() {

    }

    public void holdArmPosition() {

    }

}
