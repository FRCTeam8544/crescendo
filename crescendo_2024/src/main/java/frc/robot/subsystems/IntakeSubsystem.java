package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopConstant;

public class IntakeSubsystem {
    private static CANSparkMax rollerMotor = new CANSparkMax(Constants.IntakeConstants.RollerCANID, CANSparkLowLevel.MotorType.kBrushless);
    private static CANSparkMax armMotor = new CANSparkMax(Constants.IntakeConstants.ArmCANID, CANSparkLowLevel.MotorType.kBrushless);

    private SparkPIDController armPID = armMotor.getPIDController();

    public IntakeSubsystem() {
        rollerMotor.restoreFactoryDefaults();
        armMotor.restoreFactoryDefaults();
    
        armPID.setP(Constants.IntakeConstants.armkP);armPID.setI(Constants.IntakeConstants.armkI);armPID.setD(Constants.IntakeConstants.armkD);
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
    }
    
    public void updateDashboard(){
        SmartDashboard.putNumber("Arm Velocity", armMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Arm Position", armMotor.getEncoder().getPosition());  
    }
}
