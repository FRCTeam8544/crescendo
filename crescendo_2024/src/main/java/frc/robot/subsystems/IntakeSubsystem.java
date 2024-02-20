package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopConstant;

public class IntakeSubsystem extends SubsystemBase{
    private static CANSparkMax rollerMotor = new CANSparkMax(Constants.IntakeConstants.RollerCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax armMotor = new CANSparkMax(Constants.IntakeConstants.ArmCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static DigitalInput bobsFavoritePart = new DigitalInput(IntakeConstants.IntakeLimitSwPort); //trust - this is the limit switch
    private DigitalInput noteSensor = new DigitalInput(Constants.IntakeConstants.NoteLimitSwitchPort);
    private SparkPIDController armPID = armMotor.getPIDController();
    private AbsoluteEncoder armEncoder = armMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private double pubSet = 0.0;

    public IntakeSubsystem() {
        rollerMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults();
        armPID.setP(Constants.IntakeConstants.armkP);
        armPID.setI(Constants.IntakeConstants.armkI);
        armPID.setD(Constants.IntakeConstants.armkD);
    }

    public BooleanSupplier noteInIntake = () -> {
        return noteSensor.get();
    };

    @Override
    public void periodic(){
        if(bobsFavoritePart.get()){stop();}
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

    public void moveArm(double setpoint){
        armPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
        pubSet = setpoint;
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
