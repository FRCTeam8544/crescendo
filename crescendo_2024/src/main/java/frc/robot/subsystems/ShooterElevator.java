package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import java.util.function.BooleanSupplier;

import com.revrobotics.AbsoluteEncoder;
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

    pivotMotor.setSoftLimit(SoftLimitDirection.kForward, 60);
    pivotMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    pivotMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    pivotMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

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

    SmartDashboard.putNumber("Pivot Encoder", PivotEncoder.getPosition());

  }

  public void moveElevator(boolean up){//alex honnold would be proud
    if (!upStopRequested && up){
      shooterElevatorMotor.set(0.3);
      //rightElevatorMotor.set(-0.2);
      dirE = "fr";
    }else if (!downStopRequested && !up){
      shooterElevatorMotor.set(-0.2);
      //rightElevatorMotor.set(0.2);
      dirE = "re";
    }

  }

  public void stopElevator(){
    shooterElevatorMotor.stopMotor();
    dirE = "na";
  }

  public void movePivot(boolean out){
    if (!inStopRequested && out){
      pivotMotor.set(0.1);
      dirP = "in";
    }else if (!outStopRequested && !out){
      pivotMotor.set(-0.1);
      dirP = "out";
    }
  }

  public void movePivotWithSpeed(boolean out, double speed){
    speed = Math.abs(speed);
    if (!inStopRequested && out){
      pivotMotor.set(speed);
      dirP = "in";
    }else if (!outStopRequested && !out){
      pivotMotor.set(-1 * speed);
      dirP = "out";
    } 
  }

  public void stopPivot(){
    pivotMotor.stopMotor();
    dirP = "na";
  }

  public double getPivotEncoder(){
    return PivotEncoder.getPosition();
  }

  public double getElevatorEncoder(){
    return ElevatorEncoder.getPosition();
  }

  public void teleopElevator(boolean up, boolean down){
    if (up){moveElevator(true);}
    else if (down){moveElevator(false);}
    else{stopElevator();}
  }

}
