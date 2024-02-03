// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ShooterSubsystem;

// We probably don't need this | import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
// We probably don't need this | import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ShootSubsystem extends SubsystemBase {

  private static CANSparkMax leftMotor = new CANSparkMax(Constants.ShooterConstants.leftMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax rightMotor = new CANSparkMax(Constants.ShooterConstants.rightMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

  // We probably don't need this | private RelativeEncoder leftEncoder = leftMotor.getEncoder(); //top = left     bottom = right
  // We probably don't need this | private RelativeEncoder rightEncoder = rightMotor.getEncoder(); //shooting = left     loading = right

  // We probably don't need this | private Constants.ShooterConstants shooterConstants;

  private SparkPIDController leftMotorPID = leftMotor.getPIDController();
  private SparkPIDController rightMotorPID = rightMotor.getPIDController();

  //private PIDController shooter = new PIDController(0.000005, 5e-7, 0.0005);
  //private PIDController loader = new PIDController(0.000005, 5e-7, 0.0005);



  public ShootSubsystem() {

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    //leftMotor.follow(leftMotor, true); // Right motor is the leader

    leftMotorPID.setP(0.000005);leftMotorPID.setI(5e-7);leftMotorPID.setD(0.0005);
    rightMotorPID.setP(0.000005);rightMotorPID.setI(5e-7);rightMotorPID.setD(0.0005);

    //shooter.setOutputRange(-0.5, 0.5);loader.setOutputRange(-0.5, 0.5);
  }

  @Override
  public void periodic(){
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity()));
    //bottomMotor.set(shooter.calculate(loadingEncoder.getVelocity()));
  }

  public void sourceIntake(){
    System.out.println("sourceIntake");
    leftMotorPID.setReference(-ShooterConstants.intakeSetpoint, CANSparkBase.ControlType.kVelocity);//negative is reverse, used for intaking for the shooter
    rightMotorPID.setReference(ShooterConstants.intakeSetpoint, CANSparkBase.ControlType.kVelocity); 
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity(), -100));
    //bottomMotor.set(loader.calculate(loadingEncoder.getVelocity(), -75));
    //shooter.setSetpoint(-50);
    //loader.setSetpoint(-50);

  }

  public void shoot(){
    System.out.println("Shoot");
    leftMotorPID.setReference(ShooterConstants.shootSetpoint, CANSparkBase.ControlType.kVelocity);
    rightMotorPID.setReference(-ShooterConstants.shootSetpoint, CANSparkBase.ControlType.kVelocity);
    }
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity(), 100));
    //if (shooter.atSetpoint()){bottomMotor.set(loader.calculate(loadingEncoder.getVelocity(), 100));}
/*
    shooter.setSetpoint(2000);

    if (shooter.atSetpoint()){loader.setSetpoint(2000);}
    else{loader.setSetpoint(0);}*/
  


  public void stopMovement(){
    System.out.println("stop movement");
    leftMotorPID.setReference(ShooterConstants.stopSetpoint, CANSparkBase.ControlType.kVelocity);
    rightMotorPID.setReference(ShooterConstants.stopSetpoint, CANSparkBase.ControlType.kVelocity);
    //topMotor.set(0);
    //bottomMotor.set(0);
    //shooter.setSetpoint(0);
    //loader.setSetpoint(0);

  }
}
