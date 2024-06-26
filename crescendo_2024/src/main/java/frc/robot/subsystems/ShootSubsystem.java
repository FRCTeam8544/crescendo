// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ShootSubsystem extends SubsystemBase {

  private static CANSparkMax leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

  private SparkPIDController leftMotorPID = leftMotor.getPIDController();
  private SparkPIDController rightMotorPID = rightMotor.getPIDController();

  private DigitalInput limitSwitch = new DigitalInput(3);//optical sensor, poorly named, should be called big brother

  public ShootSubsystem() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotorPID.setP(ShooterConstants.kP);
    leftMotorPID.setI(ShooterConstants.kI);
    leftMotorPID.setD(ShooterConstants.kD);

    rightMotorPID.setP(ShooterConstants.kP);
    rightMotorPID.setI(ShooterConstants.kI);
    rightMotorPID.setD(ShooterConstants.kD);

    leftMotorPID.setOutputRange(-1, 1);
    rightMotorPID.setOutputRange(-1, 1);
  }

  public BooleanSupplier noteInShooter = () -> {
    return !limitSwitch.get();
  };

  @Override
  public void periodic(){
    //updateDashboard();
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity()));
    //bottomMotor.set(shooter.calculate(loadingEncoder.getVelocity()));

    SmartDashboard.putBoolean("Shooter Brake", leftMotor.getIdleMode() == IdleMode.kBrake);
    SmartDashboard.putBoolean("note in shooter", noteInShooter.getAsBoolean());
  }

  /*  
    The way that the shooter works currently, and will likely work, 
    we don't need to intake via shooter

  public void sourceIntake(double setpoint){
    System.out.println("sourceIntake");
    leftMotorPID.setReference(-setpoint, CANSparkBase.ControlType.kVelocity);//negative is reverse, used for intaking for the shooter
    rightMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity); 
  }*/

  public void setBrake(){
    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);
  }

  public void setZero(){
    leftMotorPID.setReference(leftMotor.getEncoder().getPosition(), CANSparkBase.ControlType.kPosition);
    rightMotorPID.setReference(rightMotor.getEncoder().getPosition(), CANSparkBase.ControlType.kPosition);
  }

  public void setCoast(){
    leftMotor.setIdleMode(IdleMode.kCoast);
    rightMotor.setIdleMode(IdleMode.kCoast);
  }

  public void handoff(){
    leftMotor.set(0.25);
    rightMotor.set(-0.25);
  }

  public void sourceIntake(){
    leftMotor.set(-0.25);
    rightMotor.set(0.25);
  }

  public void shoot(double setpoint){
    leftMotorPID.setReference(setpoint, CANSparkBase.ControlType.kVelocity);
    rightMotorPID.setReference(-setpoint, CANSparkBase.ControlType.kVelocity);
  }

  public void stop(){
    leftMotorPID.setReference(0, CANSparkBase.ControlType.kVelocity);
    rightMotorPID.setReference(0, CANSparkBase.ControlType.kVelocity);
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Left Motor Velocity", leftMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Right Motor Velocity", rightMotor.getEncoder().getVelocity());  
  }

  public double getRightVelocity(){
    return rightMotor.getEncoder().getVelocity();
  }

  public BooleanSupplier atSpeed = () -> {
    return getLeftVelocity() < 5000 + 100 && getLeftVelocity() > 5000 - 100;
  };

  public boolean booleanSpeed(double speed){
    return  getLeftVelocity() < speed + 100 && getLeftVelocity() > speed - 100;
  }

  public BooleanSupplier atTestSpeed = () -> {
    return (getLeftVelocity() < 100 + 50 && getLeftVelocity() > 100 - 50);
  };

  public double getLeftVelocity(){
    return leftMotor.getEncoder().getVelocity();
  }
}
