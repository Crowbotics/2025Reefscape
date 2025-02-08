package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class ClawSubsystem extends SubsystemBase{

    private final SparkMax m_topCollector = new SparkMax(ClawConstants.kTopPort, MotorType.kBrushless);
    private final SparkMax m_bottomCollector = new SparkMax(ClawConstants.kBottomPort, MotorType.kBrushless);
    private final SparkMax m_leftManipulator = new SparkMax(ClawConstants.kLeftPort, MotorType.kBrushless);
    private final SparkMax m_rightManipulator = new SparkMax(ClawConstants.kRightPort, MotorType.kBrushless);
    private final SparkMax m_kicker = new SparkMax(ClawConstants.kKickerPort, MotorType.kBrushless);
    
    private final DigitalInput[] m_sensors = {
        new DigitalInput(ClawConstants.kCoralSensor0),
        new DigitalInput(ClawConstants.kCoralSensor1),
        new DigitalInput(ClawConstants.kCoralSensor2),
        new DigitalInput(ClawConstants.kCoralSensor3)
    };
    // Sensor states left to right from the robot's perspective
    private final boolean[] m_sensorStates = {false, false, false, false};

    public ClawSubsystem(){

    }
    
    @Override
    public void periodic() {
        // Update sensor states
        for (int i = 0; i < m_sensorStates.length; i++) {
            m_sensorStates[i] = m_sensors[i].get();
            SmartDashboard.putBooleanArray("Coral Sensor States", m_sensorStates);
        }
    }

    public void intake() {
        m_topCollector.set(ClawConstants.kCollectorSpeed);
        m_bottomCollector.set(ClawConstants.kCollectorSpeed);

    }

    public void reverseIntake() {
        m_topCollector.set(ClawConstants.kReverseCollectorSpeed);
        m_bottomCollector.set(ClawConstants.kReverseCollectorSpeed);

    }

    public void centerCoral() {

    }

    public void ejectCoralLeft() {

    }

    public void ejectCoralRight() {
        
    }
    
}
