// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;

public class ShootSubsystem extends SubsystemBase {

  private static CANSparkMax topMotor = new CANSparkMax(Constants.ShooterConstants.TopMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax bottomMotor = new CANSparkMax(Constants.ShooterConstants.BottomMotorCANID, CANSparkLowLevel.MotorType.kBrushless);

  private RelativeEncoder shootingEncoder = topMotor.getEncoder();
  private RelativeEncoder loadingEncoder = bottomMotor.getEncoder();

  private Constants.ShooterConstants shooterConstants;

  private SparkPIDController shooter = topMotor.getPIDController();
  private SparkPIDController loader = bottomMotor.getPIDController();

  //private PIDController shooter = new PIDController(0.000005, 5e-7, 0.0005);
  //private PIDController loader = new PIDController(0.000005, 5e-7, 0.0005);



  public ShootSubsystem() {

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();


    shooter.setD(0.0005);shooter.setI(5e-7);shooter.setP(0.000005);
    loader.setD(0.0005);loader.setI(5e-7);loader.setP(0.000005);

    //shooter.setOutputRange(-0.5, 0.5);loader.setOutputRange(-0.5, 0.5);


  }

  @Override
  public void periodic(){
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity()));
    //bottomMotor.set(shooter.calculate(loadingEncoder.getVelocity()));
  }

  public void loadShooter() {
    topMotor.set(-.1);
    bottomMotor.set(-.1);
  }


  public void sourceIntake(){
    System.out.println("sourceIntake");
    shooter.setReference(-100, CANSparkBase.ControlType.kVelocity);//negative is reverse, used for intaking for the shooter
    loader.setReference(-100, CANSparkBase.ControlType.kVelocity); 
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity(), -100));
    //bottomMotor.set(loader.calculate(loadingEncoder.getVelocity(), -75));
    //shooter.setSetpoint(-50);
    //loader.setSetpoint(-50);

  }

  public void shoot(){
    System.out.println("Shoot");
    shooter.setReference(5000, CANSparkBase.ControlType.kVelocity);
    if (shootingEncoder.getVelocity() > 4600){
      loader.setReference(5000, CANSparkBase.ControlType.kVelocity);
    }
    //topMotor.set(shooter.calculate(shootingEncoder.getVelocity(), 100));
    //if (shooter.atSetpoint()){bottomMotor.set(loader.calculate(loadingEncoder.getVelocity(), 100));}
/*
    shooter.setSetpoint(2000);

    if (shooter.atSetpoint()){loader.setSetpoint(2000);}
    else{loader.setSetpoint(0);}*/
  }


  public void stopMovement(){
    System.out.println("stop movement");
    shooter.setReference(0, CANSparkBase.ControlType.kVelocity);
    loader.setReference(0, CANSparkBase.ControlType.kVelocity);
    //topMotor.set(0);
    //bottomMotor.set(0);
    //shooter.setSetpoint(0);
    //loader.setSetpoint(0);

  }

  public void fireInTheHole() {
    bottomMotor.set(1);
  }

  public void prep() { 
    topMotor.set(1);
  }

  public void stopIt()
  {
    topMotor.set(0);
    bottomMotor.set(0);
  }
}
