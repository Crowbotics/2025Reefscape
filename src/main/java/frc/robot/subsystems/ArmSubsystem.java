package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase{

    public final SparkMax m_pivotMotor1 = new SparkMax(Constants.ArmConstants.kArmPort1, MotorType.kBrushless);
    public final SparkMax m_pivotMotor2 = new SparkMax(Constants.ArmConstants.kArmPort2, MotorType.kBrushless);

    public final SparkClosedLoopController m_armController = m_pivotMotor1.getClosedLoopController();
    public final SparkAbsoluteEncoder m_armEncoder = m_pivotMotor1.getAbsoluteEncoder();

    public final SparkMaxConfig m_pivotMotor1Config  = new SparkMaxConfig();
    public final SparkMaxConfig m_pivotMotor2Config = new SparkMaxConfig();

    public enum ArmState {
        GROUND_FRONT    (ArmConstants.kGroundFront),
        L1_FRONT        (ArmConstants.kL1Front),
        L2_FRONT        (ArmConstants.kL2Front),
        STRAIGHT_UP     (ArmConstants.kStraightUp),
        L2_REAR         (ArmConstants.kL2Rear),
        L1_REAR         (ArmConstants.kL1Rear),
        GROUND_REAR     (ArmConstants.kGroundRear);

        public final double position;
        public ArmState next;
        public ArmState prev;

        ArmState(double pos)
        {
            this.position = pos;
        }

        //Sets up how we iterate through arm states
        static{
            GROUND_FRONT.next = L1_FRONT;       GROUND_FRONT.prev = GROUND_FRONT;
            L1_FRONT.next =     L2_FRONT;       L1_FRONT.prev =     GROUND_FRONT;
            L2_FRONT.next =     STRAIGHT_UP;    L2_FRONT.prev =     L1_FRONT;
            STRAIGHT_UP.next =  STRAIGHT_UP; /*was L2_REAR*/        STRAIGHT_UP.prev = L2_FRONT;
            L2_REAR.next =      L1_REAR;        L2_REAR.prev =      STRAIGHT_UP;
            L1_REAR.next =      GROUND_REAR;    L1_REAR.prev =      L2_REAR;
            GROUND_REAR.next =  GROUND_REAR;    GROUND_REAR.prev =  L1_REAR;
        }

        public ArmState next() { return this.next;}
        public ArmState prev() { return this.prev;}
    }

    public ArmState m_state;
    
    public ArmSubsystem() {

        m_state = ArmState.STRAIGHT_UP;

        //Set Leader
        m_pivotMotor1Config
            .idleMode(IdleMode.kBrake);

        m_pivotMotor1Config.encoder
            .positionConversionFactor(1);

        m_pivotMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(   ArmConstants.kP,
                    ArmConstants.kI,
                    ArmConstants.kD);

        m_pivotMotor1.configure(m_pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Set Follower
        m_pivotMotor2Config.follow(m_pivotMotor1, true);
        m_pivotMotor2.configure(m_pivotMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){
        SmartDashboard.putNumber("Arm Position: ", getArmPosition());
        SmartDashboard.putString("Arm State: ", m_state.toString());
    }

    public Command moveArmForward(){
        return Commands.runOnce(this::nextArmState, this);
    }

    public Command moveArmBackward(){
        return Commands.runOnce(this::prevArmState, this);
    }

    public Command moveArmCommand(){
        return new RunCommand(this::setArmReference, this);
    }

    private void setArmReference(){
        m_pivotMotor1.getClosedLoopController().setReference(m_state.position, ControlType.kPosition);
    }

    private void nextArmState() {
        m_state = m_state.next();
    }

    private void prevArmState() {
        m_state = m_state.prev();
    }

    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }


}
