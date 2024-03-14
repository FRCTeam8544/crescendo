package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimbElevatorConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.StopConstant;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ClimberElevator extends SubsystemBase {

  private static CANSparkMax elevatorMotor = new CANSparkMax(Constants.ClimbElevatorConstants.LeftElevatorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax rightElevatorMotor = new CANSparkMax(Constants.ClimbElevatorConstants.RightElevatorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static DigitalInput upLimit = new DigitalInput(ClimbElevatorConstants.UpLimitPort);
  private static DigitalInput downLimit = new DigitalInput(ClimbElevatorConstants.DownLimitPort);
  //private SparkPIDController elevatorMotorPID = elevatorMotor.getPIDController();
  private RelativeEncoder elevatorEncoder = elevatorMotor.getAlternateEncoder(ClimbElevatorConstants.CountsPerRev);
  private boolean upStopRequested = false;
  private boolean downStopRequested = true;

  private String dir = "na";

  public BooleanSupplier noUp = () -> {//can we get much higher
    return !upLimit.get();//so high
  };//so high
  public BooleanSupplier noDown = () -> {//diver dung
    return !downLimit.get();
  };

  public ClimberElevator() {
    

    elevatorMotor.restoreFactoryDefaults();// I <3 NYC
    rightElevatorMotor.restoreFactoryDefaults();// I !<3 LA

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    //elevatorMotorPID.setFeedbackDevice(elevatorEncoder);
    rightElevatorMotor.follow(elevatorMotor, true);//not all who wander are lost

    elevatorMotor.burnFlash();
    rightElevatorMotor.burnFlash();

    /*elevatorMotorPID.setP(ClimbElevatorConstants.elevkP);
    elevatorMotorPID.setI(ClimbElevatorConstants.elevkI);
    elevatorMotorPID.setD(ClimbElevatorConstants.elevkD);*/
  }

  @Override
  public void periodic(){//T = 1/f
    if (noUp.getAsBoolean()){ //|| elevatorEncoder.getPosition() > 40){
      upStopRequested = true;
    }else{
      upStopRequested = false;
    }
    
    if (noDown.getAsBoolean()){// || elevatorEncoder.getPosition() < 2){
      downStopRequested = true;
    }else{
      downStopRequested = false;
    }
    
    if (downStopRequested && dir == "re"){
      elevatorMotor.set(StopConstant.stopSetpoint);
    }else if (upStopRequested && dir == "fr"){
      elevatorMotor.set(StopConstant.stopSetpoint);
    }
    updateDashboard();
  }

  public void moveClimber(boolean up){//alex honnold would be proud
    if (!upStopRequested && up){
      elevatorMotor.set(ClimbElevatorConstants.elevatorPercentMove);
      dir = "fr";
    }else if (!downStopRequested && !up){
      elevatorMotor.set(-ClimbElevatorConstants.elevatorPercentMove);
      dir = "re";
    }

  }

  public void stop(){//double setpoint){
    //elevatorMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    elevatorMotor.set(StopConstant.stopSetpoint);
    dir = "na";
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Left climber Velocity", elevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("right Climber Speed", rightElevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber Position", elevatorMotor.getEncoder().getPosition());  
  }

  public double getElevatorVelocity(){
    return elevatorMotor.getEncoder().getVelocity();
  }

  public double getElevatorPosition(){
    return elevatorEncoder.getPosition();
  }
}
