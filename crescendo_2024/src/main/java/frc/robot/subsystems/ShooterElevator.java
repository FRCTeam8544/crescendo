package frc.robot.subsystems;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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

public class ShooterElevator extends SubsystemBase {

  private static CANSparkMax shooterElevatorMotor = new CANSparkMax(Constants.ShootElevatorConstants.ElevatorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax pivotMotor = new CANSparkMax(Constants.ShootElevatorConstants.PivotCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static DigitalInput upLimit = new DigitalInput(7);
  private static DigitalInput downLimit = new DigitalInput(6);
  private static DigitalInput outLimit = new DigitalInput(5);
  private static DigitalInput inLimit = new DigitalInput(4);
  //private SparkPIDController elevatorMotorPID = elevatorMotor.getPIDController();
  private RelativeEncoder ElevatorEncoder = shooterElevatorMotor.getEncoder();
  private AbsoluteEncoder PivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private boolean upStopRequested = false;
  private boolean downStopRequested = true;
  private boolean outStopRequested = false;
  private boolean inStopRequested = false;

  private String dirE = "na";
  private String dirP = "na";

  public BooleanSupplier noUp = () -> {//can we get much higher
    return !upLimit.get();//so high
  };//so high
  public BooleanSupplier noDown = () -> {//diver dung
    return !downLimit.get();
  };
  public BooleanSupplier noOut = () -> {
    return !outLimit.get();
  };
  public BooleanSupplier noIn = () -> {
    return !inLimit.get();
  };

  public ShooterElevator() {
    

    shooterElevatorMotor.restoreFactoryDefaults();
    pivotMotor.restoreFactoryDefaults();
    

    shooterElevatorMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);

    shooterElevatorMotor.burnFlash();
    pivotMotor.burnFlash();


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

    if (noIn.getAsBoolean()){
      inStopRequested = true;
    }else{
      inStopRequested = false;
    }

    if (noOut.getAsBoolean()){
      outStopRequested = true;
    }else{
      outStopRequested = false;
    }
    
    if (downStopRequested && dirE == "re"){
      shooterElevatorMotor.set(0);
      //rightElevatorMotor.set(0);
    }else if (upStopRequested && dirE == "fr"){
      shooterElevatorMotor.set(0);
      //rightElevatorMotor.set(0);
    }

    if (inStopRequested && dirP == "in"){
      pivotMotor.set(0);
    }else if (outStopRequested && dirP == "out"){
      pivotMotor.set(0);
    }

  }

  public void moveElevator(boolean up){//alex honnold would be proud
    if (!upStopRequested && up){
      shooterElevatorMotor.set(0.2);
      //rightElevatorMotor.set(-0.2);
      dirE = "fr";
    }else if (!downStopRequested && !up){
      shooterElevatorMotor.set(-0.2);
      //rightElevatorMotor.set(0.2);
      dirE = "re";
    }

  }

  public void stopElevator(){
    shooterElevatorMotor.set(0);
    dirE = "na";
  }

  public void movePivor(boolean in){
    if (!inStopRequested && in){
      pivotMotor.set(0.2);
      dirP = "in";
    }else if (!outStopRequested && !in){
      pivotMotor.set(-0.2);
      dirP = "out";
    }
  }

  public void stopPivot(){
    pivotMotor.set(0);
    dirP = "na";
  }

}
