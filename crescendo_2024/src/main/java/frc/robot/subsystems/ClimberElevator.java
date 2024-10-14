package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbElevatorConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkLowLevel;

public class ClimberElevator extends SubsystemBase {

  private static CANSparkMax leftElevatorMotor = new CANSparkMax(ClimbElevatorConstants.LeftElevatorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax rightElevatorMotor = new CANSparkMax(ClimbElevatorConstants.RightElevatorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static DigitalInput upLimit = new DigitalInput(ClimbElevatorConstants.upLimChannel);
  private static DigitalInput downLimit = new DigitalInput(ClimbElevatorConstants.downLimChannel);
  private RelativeEncoder leftElevatorEncoder = leftElevatorMotor.getAlternateEncoder(ClimbElevatorConstants.countsPerRevTarget);
  private boolean upStopRequested = false;
  private boolean downStopRequested = true;

  private String direction = "n/a";

  public BooleanSupplier noUp = () -> {//can we get much higher
    return !upLimit.get();//so high
  };//so high
  public BooleanSupplier noDown = () -> {//diver dung
    return !downLimit.get();
  };

  public ClimberElevator() {
    leftElevatorMotor.restoreFactoryDefaults();// I <3 NYC
    rightElevatorMotor.restoreFactoryDefaults();// I !<3 LA

    leftElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.setIdleMode(IdleMode.kBrake);
    rightElevatorMotor.follow(leftElevatorMotor, true);//not all who wander are lost

    leftElevatorMotor.burnFlash();//never forget the holy scriptures
    rightElevatorMotor.burnFlash();
  }

  @Override
  public void periodic(){
    if (noUp.getAsBoolean())
      upStopRequested = true;
    else
      upStopRequested = false;
    
    if (noDown.getAsBoolean())
      downStopRequested = true;
    else
      downStopRequested = false;
    
    if (downStopRequested && direction == "reverse")
      leftElevatorMotor.set(0);
    else if (upStopRequested && direction == "forward")
      leftElevatorMotor.set(0);
    
    updateDashboard();
  }

  public void moveClimber(boolean up){//alex honnold would be proud
    if (!upStopRequested && up){
      leftElevatorMotor.set(ClimbElevatorConstants.motorSpeed);
      direction = "forward";
    }else if (!downStopRequested && !up){
      leftElevatorMotor.set(-ClimbElevatorConstants.motorSpeed);
      direction = "reverse";
    }
  }

  public void stop(){
    leftElevatorMotor.set(0);
    direction = "n/a";
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Left climber Velocity", leftElevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Climber Speed", rightElevatorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Climber Position", leftElevatorMotor.getEncoder().getPosition());  
  }

  public double getElevatorVelocity(){
    return leftElevatorMotor.getEncoder().getVelocity();
  }

  public double getElevatorPosition(){
    return leftElevatorEncoder.getPosition();
  }
}
