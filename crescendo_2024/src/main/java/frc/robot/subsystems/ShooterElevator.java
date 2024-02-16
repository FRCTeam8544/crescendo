package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShootElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ShooterElevator extends SubsystemBase {

  private static CANSparkMax elevatorMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax pivotMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

  private SparkPIDController elevatorMotorPID = elevatorMotor.getPIDController();
  private SparkPIDController pivotMotorPID = pivotMotor.getPIDController();

  public ShooterElevator() {

    elevatorMotor.restoreFactoryDefaults();
    pivotMotor.restoreFactoryDefaults();

    elevatorMotorPID.setP(ShootElevatorConstants.elevkP);
    elevatorMotorPID.setI(ShootElevatorConstants.elevkI);
    elevatorMotorPID.setD(ShootElevatorConstants.elevkD);

    pivotMotorPID.setP(ShootElevatorConstants.pivotkP);
    pivotMotorPID.setI(ShootElevatorConstants.pivotkI);
    pivotMotorPID.setD(ShootElevatorConstants.pivotkD);
  }

  @Override
  public void periodic(){
    updateDashboard();
  }

  public void muévete(double setpoint){
    //the setpoint can be pos or neg for up and down movement respectively
    //TODO implement limit switch for safety
    elevatorMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void rotatePivot(double setpoint){
    //the setpoint can be pos or neg for up and down movement respectively
    //TODO implement limit switch for safety
    pivotMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void stopElevator(double setpoint){
    elevatorMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void stopPivot(double setpoint){
    pivotMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }


  public void updateDashboard(){ 
    SmartDashboard.putNumber("Elevator Velocity", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Elevator Position", elevatorMotor.getEncoder().getPosition());  
    SmartDashboard.putNumber("Pivot Velocity", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Pivot Position", elevatorMotor.getEncoder().getPosition());  
  }

  public double getElevatorVelocity(){
    return elevatorMotor.getEncoder().getVelocity();
  }

  public double getElevatorPosition(){
    return elevatorMotor.getEncoder().getPosition();
  }
}
