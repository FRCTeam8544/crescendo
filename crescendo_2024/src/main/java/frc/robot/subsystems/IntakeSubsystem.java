package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopConstant;

public class IntakeSubsystem extends SubsystemBase{
    private static CANSparkMax rollerMotor = new CANSparkMax(Constants.IntakeConstants.RollerCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax armMotor = new CANSparkMax(Constants.IntakeConstants.ArmCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static DigitalInput forwardLimit = new DigitalInput(IntakeConstants.IntakeLimitSwPort);
    private static DigitalInput reverseLimit = new DigitalInput(0);
    private DigitalInput noteSensor = new DigitalInput(Constants.IntakeConstants.NoteLimitSwitchPort);
    private SparkPIDController armPID;
    private AbsoluteEncoder armEncoder;
    private double pubSet;
    private boolean forwardStopRequested = false;
    private boolean reverseStopRequested = false;
    private String dir = "na";
    private boolean noteInside = false;

    public IntakeSubsystem() {
        rollerMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults();

        rollerMotor.setIdleMode(IdleMode.kBrake);

        armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        /*armPID = armMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);
        
        armPID.setP(0.02);
        armPID.setI(1e-7);
        armPID.setD(0.002);
        armPID.setFF(0);
        armPID.setOutputRange(-0.5, 0.5);*/
        armMotor.setIdleMode(IdleMode.kCoast);

        //armMotor.setSoftLimit(SoftLimitDirection.kForward, 25);
        //armMotor.setSoftLimit(SoftLimitDirection.kReverse, -3);

        pubSet = armEncoder.getPosition();

        armMotor.burnFlash();
    }

    public BooleanSupplier noteInIntake = () -> {
        return noteSensor.get();
    };

    public BooleanSupplier forwardLimitSwitch = () -> {
        return !forwardLimit.get();
    };
    public BooleanSupplier reverseLimitSwitch = () -> {
        return !reverseLimit.get();
    };

    @Override
    public void periodic(){
        if (forwardLimitSwitch.getAsBoolean()){
            forwardStopRequested = true;
        }else{
            forwardStopRequested = false;
        }if (reverseLimitSwitch.getAsBoolean()){
            reverseStopRequested = true;
        }else{
            reverseStopRequested = false;
        }
        if (reverseStopRequested && dir == "re"){
            armMotor.set(0);
        }else if (forwardStopRequested && dir == "fr"){
            armMotor.set(0);
        }

        if (noteInIntake.getAsBoolean()){noteInside = true;}
        else{noteInside = false;}
        /*else{
            armPID.setReference(pubSet, CANSparkMax.ControlType.kPosition);
        }*/
        updateDashboard();
    }

    public void suckySuck(){
        if(noteInside){rollerMotor.set(IntakeConstants.suckySuckSpeed);}
        else{rollerMotor.set(0);}
    }

    public void feedTheMachine(){
        rollerMotor.set(IntakeConstants.rateMachineIsFed);
    }
    
    public void rageAgainsTheMachine(){
        rollerMotor.set(0.2);
    }

    public void stop(){
        rollerMotor.set(StopConstant.stopSetpoint);
    }

    public void rotateStop(){
        armMotor.set(0);
        pubSet = armEncoder.getPosition();
        dir = "na";
    }

    public void moveArm(double setpoint){
        pubSet = setpoint;
    }

    public void testRotate(boolean forward){
        if (!forwardStopRequested && forward){
           armMotor.set(0.20); 
           dir = "fr";
        }else if (!reverseStopRequested && !forward){
            armMotor.set(-0.15);
            dir = "re";
        }
    }

    public BooleanSupplier atSetpoint = () -> {
        return (armEncoder.getPosition() <= pubSet + 0.1 && armEncoder.getPosition() >= pubSet - 0.1);
    };
    
    public void updateDashboard(){
        SmartDashboard.putNumber("Arm Velocity", armMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());
    }

    public double getArmPosition(){
       return armMotor.getEncoder().getPosition();
    }

    public double getArmVelocity(){
        return armMotor.getEncoder().getVelocity();
    }
}
