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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
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
        GROUND_REAR     (ArmConstants.kGroundRear),
        BEGIN_CLIMB     (ArmConstants.kBeginClimb),
        END_CLIMB       (ArmConstants.kEndClimb);


        public final double position;
        public ArmState up;
        public ArmState down;

        ArmState(double pos)
        {
            this.position = pos;
        }

        //Sets up how we iterate through arm states
        static{
            GROUND_FRONT.up = L1_FRONT; GROUND_FRONT.down = GROUND_FRONT;
            L1_FRONT.up =     L2_FRONT; L1_FRONT.down =     GROUND_FRONT;
            L2_FRONT.up =     L2_REAR;  L2_FRONT.down =     L1_FRONT;
            STRAIGHT_UP.up =  L2_REAR;  STRAIGHT_UP.down = L2_FRONT;
            L2_REAR.up =      L2_FRONT; L2_REAR.down =      L1_REAR;
            L1_REAR.up =      L2_REAR;  L1_REAR.down =      GROUND_REAR;
            GROUND_REAR.up =  L1_REAR;  GROUND_REAR.down =  GROUND_REAR;
        }

        public ArmState up() { return this.up;}
        public ArmState down() { return this.down;}
    }

    public ArmState m_state;
    
    public ArmSubsystem() {

        m_state = ArmState.L1_FRONT;

        //Set Leader
        m_pivotMotor1Config
            .idleMode(IdleMode.kBrake);

        m_pivotMotor1Config.encoder
            .positionConversionFactor(1);

        m_pivotMotor1Config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pidf(   ArmConstants.kP,
                    ArmConstants.kI,
                    ArmConstants.kD,
                    ArmConstants.kFF);

        m_pivotMotor1.configure(m_pivotMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Set Follower
        m_pivotMotor2Config.follow(m_pivotMotor1, true);
        m_pivotMotor2.configure(m_pivotMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void periodic(){
        SmartDashboard.putNumber("Arm Position: ", getArmPosition());
        SmartDashboard.putString("Arm State: ", m_state.toString());
    }

    public Command moveArmCommand(){
        return new RunCommand(this::setArmReference, this);
    }

    public Command moveArmStateDown(){
        return Commands.runOnce(this::setArmStateDown, this);
    }

    public Command moveArmStateUp(){
        return Commands.runOnce(this::setArmStateUp, this);
    }

    public Command moveArmStateToBeginClimbCommand() {
        return Commands.runOnce(this::setArmStateBeginClimb);
    }

    public Command moveArmStateToEndClimbCommand() {
        return Commands.runOnce(this::setArmStateEndClimb);
    }

    private void setArmReference(){
        m_pivotMotor1.getClosedLoopController().setReference(m_state.position, ControlType.kPosition);
    }

    private void setArmStateUp() {
        m_state = m_state.up();
    }

    private void setArmStateDown() {
        m_state = m_state.down();
    }

    private void setArmStateBeginClimb() {
        m_state = ArmState.BEGIN_CLIMB;
    }

    private void setArmStateEndClimb() {
        m_state = ArmState.END_CLIMB;
    }

    public double getArmPosition() {
        return m_armEncoder.getPosition();
    }

    public boolean isArmAtSetpoint()
    {
        return Math.abs(m_pivotMotor1.getEncoder().getPosition() - m_state.position) < .05;
    }


}
