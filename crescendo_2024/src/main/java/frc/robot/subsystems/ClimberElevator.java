package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ClimberElevator extends SubsystemBase {

  private static CANSparkMax elevatorMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax rightElevatorMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

  private SparkPIDController elevatorMotorPID = elevatorMotor.getPIDController();
  private AbsoluteEncoder elevatorEncoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public ClimberElevator() {

    elevatorMotor.restoreFactoryDefaults();
    rightElevatorMotor.restoreFactoryDefaults();
    elevatorMotorPID.setFeedbackDevice(elevatorEncoder);

    rightElevatorMotor.follow(elevatorMotor, true);

    elevatorMotorPID.setP(ClimbElevatorConstants.elevkP);
    elevatorMotorPID.setI(ClimbElevatorConstants.elevkI);
    elevatorMotorPID.setD(ClimbElevatorConstants.elevkD);
  }

  @Override
  public void periodic(){
    updateDashboard();
  }

  public void moveClimber(double setpoint){
    //the setpoint can be pos or neg for up and down movement respectively
    //TODO implement limit switch for safety
    elevatorMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void stop(double setpoint){
    elevatorMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Climber Velocity", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber Position", elevatorMotor.getEncoder().getPosition());  
  }

  public double getElevatorVelocity(){
    return elevatorMotor.getEncoder().getVelocity();
  }

  public double getElevatorPosition(){
    return elevatorMotor.getEncoder().getPosition();
  }
}
