package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopConstant;

public class IntakeSubsystem extends SubsystemBase{
    private static CANSparkMax rollerMotor = new CANSparkMax(Constants.IntakeConstants.RollerCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax armMotor = new CANSparkMax(Constants.IntakeConstants.ArmCANID, CANSparkLowLevel.MotorType.kBrushless);
    //private static DigitalInput bobsFavoritePart = new DigitalInput(IntakeConstants.IntakeLimitSwPort); //trust - this is the limit switch
    //private DigitalInput noteSensor = new DigitalInput(Constants.IntakeConstants.NoteLimitSwitchPort);
    private SparkPIDController armPID;
    private AbsoluteEncoder armEncoder;
    private double pubSet;

    public IntakeSubsystem() {
        rollerMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults();

        armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
        armPID = armMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);
        
        armPID.setP(0.02);
        armPID.setI(1e-7);
        armPID.setD(0.002);
        armPID.setFF(0);
        armPID.setOutputRange(-0.5, 0.5);
        armMotor.setIdleMode(IdleMode.kCoast);

        pubSet = armEncoder.getPosition();

        armMotor.burnFlash();
    }

    /*public BooleanSupplier noteInIntake = () -> {
        return noteSensor.get();
    };*/

    @Override
    public void periodic(){
        //if(bobsFavoritePart.get()){stop();}
        armPID.setReference(pubSet, CANSparkMax.ControlType.kPosition);
        updateDashboard();
    }

    public void suckySuck(){
        rollerMotor.set(IntakeConstants.suckySuckSpeed);
    }

    public void feedTheMachine(){
        rollerMotor.set(IntakeConstants.rateMachineIsFed);
    }

    public void stop(){
        rollerMotor.set(StopConstant.stopSetpoint);
    }

    public void rotateStop(){
        armMotor.set(0);
        pubSet = armEncoder.getPosition();
    }

    public void moveArm(double setpoint){
        pubSet = setpoint;
    }

    public void testRotate(boolean forward){
        armMotor.set(forward? 0.2: -0.2);
    }

    public BooleanSupplier atSetpoint = () -> {
        return (armEncoder.getPosition() == pubSet);
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
