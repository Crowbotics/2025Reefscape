package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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

    public ClawSubsystem() {
        SmartDashboard.putBoolean("Outtaking Extra Coral", false);
        SmartDashboard.putString("Outtake Extra Coral Direction", "None");
    }
    
    @Override
    public void periodic() {
        // Update sensor states
        for (int i = 0; i < m_sensorStates.length; i++) {
            m_sensorStates[i] = m_sensors[i].get();
            SmartDashboard.putBoolean("Coral Sensor " + i, m_sensorStates[i]);
        }
    }

    // Returns the number of sensors that detect coral
    private int numberOfSensorsActive() {
        int sensorsActive = 0;
        for (int i = 0; i < m_sensorStates.length; i++) {
            if (m_sensorStates[i] == true) {
                sensorsActive += 1;
            }
        }
        return sensorsActive;
    }
    
    // Compares a boolean array to what the sensors detect
    // Returns true if the array matches
    private boolean sensorsMatch(boolean[] states) {
        for (int i = 0; i < m_sensorStates.length; i++) {
            if (states[i] != m_sensorStates[i]) {
                return false;
            }
        }
        return true;
    }

    public boolean[] getCoralSensorStates() {
        return m_sensorStates;
    }

    private String outtakeDirection = "";
    public Command outtakeExtraCoral() {
        return new FunctionalCommand(() -> {
            SmartDashboard.putBoolean("Outtaking Extra Coral", true);

            if (// Scenarios where we want to outtake to the right
                sensorsMatch(new boolean[]{true, true, false, true}) ||
                sensorsMatch(new boolean[]{false, true, true, true}) ||
                sensorsMatch(new boolean[]{true, true, true, true})) {
                    SmartDashboard.putString("Outtake Extra Coral Direction", "Right");
                    outtakeDirection = "Right";

            } else if ( // Scenarios where we want to outtake to the left
                sensorsMatch(new boolean[]{true, false, true, true}) ||
                sensorsMatch(new boolean[]{true, true, true, false})) {
                    SmartDashboard.putString("Outtake Extra Coral Direction", "Left");
                    outtakeDirection = "Left";
            }
        },
        () -> {
            if (outtakeDirection.equals("Right")) {
                m_rightManipulator.set(ClawConstants.kManipulatorSpeed);
            } else if (outtakeDirection.equals("Left")) {
                m_leftManipulator.set(ClawConstants.kManipulatorSpeed);
            }
        }, (Boolean interrupted) -> {
            m_leftManipulator.set(0);
            m_rightManipulator.set(0);
            SmartDashboard.putBoolean("Outtaking Extra Coral", false);
            SmartDashboard.putString("Outtake Extra Coral Direction", "None");
        }, () -> {
            if (numberOfSensorsActive() <= 2) {
                return true;
            }
            return false;
        }, this)
        
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command centerCoral() {
        return new FunctionalCommand(() -> {
            if (m_sensorStates[0] == true) {
                m_leftManipulator.set(-ClawConstants.kManipulatorSpeed);
            } else if (this.m_sensorStates[3] == true) {
                m_rightManipulator.set(-ClawConstants.kManipulatorSpeed);
            }
            SmartDashboard.putBoolean("CenterCoral Running", true);
        }, () -> {}, (Boolean interrupted) -> {
            m_leftManipulator.set(0);
            m_rightManipulator.set(0);
            SmartDashboard.putBoolean("CenterCoral Running", false);
        }, () -> {
            if (m_sensorStates[1] == true && m_sensorStates[2] == true) {
                return true;
            }
            return false;
        }, this)

        // Commands let other commands interrupt by default,
        // but this is here for clarity
        .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }

    public Command intake() {
        return new StartEndCommand(
            () -> setCollectorMotors(ClawConstants.kCollectorSpeed, -ClawConstants.kCollectorSpeed, false),
            () -> setCollectorMotors(0, 0, false),
            this);
    }

    public Command reverseIntake() {
        return new StartEndCommand(
            () -> setCollectorMotors(-ClawConstants.kCollectorSpeed, ClawConstants.kCollectorSpeed, true),
            () -> setCollectorMotors(0, 0, false),
            this);
    }

    public Command scoreLeft() {
        return new StartEndCommand(
            () -> setManipulatorMotors(ClawConstants.kManipulatorSpeed, -ClawConstants.kManipulatorSpeed),
            () -> setManipulatorMotors(0, 0),
            this);
    }

    public Command scoreRight() {
        return new StartEndCommand(
            () -> setManipulatorMotors(-ClawConstants.kManipulatorSpeed, ClawConstants.kManipulatorSpeed),
            () -> setManipulatorMotors(0, 0),
            this);
    }

    private void setCollectorMotors(double top, double bottom, boolean reverse)
    {
        m_topCollector.set(top);
        m_bottomCollector.set(bottom);
        if(reverse){
            m_kicker.set(0.3);
        } else {
            m_kicker.set(0);
        }
    }

    private void setManipulatorMotors(double left, double right)
    {
        m_leftManipulator.set(left);
        m_rightManipulator.set(right);
    }
    
}
