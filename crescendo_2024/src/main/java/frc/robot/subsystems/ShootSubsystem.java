// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

  private SparkPIDController shooter = topMotor.getPIDController();
  private SparkPIDController loader = bottomMotor.getPIDController();



  public ShootSubsystem() {

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();


    shooter.setD(0);shooter.setI(0);shooter.setP(1);
    loader.setD(0);loader.setI(0);loader.setP(1);

    shooter.setOutputRange(-0.5, 0.5);loader.setOutputRange(-0.5, 0.5);

  }

  public void loadShooter() {
    topMotor.set(-.1);
    bottomMotor.set(-.1);
  }


  public void sourceIntake(){
    System.out.println("sourceIntake");
    shooter.setReference(-100, CANSparkBase.ControlType.kVelocity);//negative is reverse, used for intaking for the shooter
    loader.setReference(-75, CANSparkBase.ControlType.kVelocity); 
  }

  public void shoot(){
    System.out.println("Shoot");
    shooter.setReference(1000, CANSparkBase.ControlType.kVelocity);
    if (shootingEncoder.getVelocity() > 190){
      loader.setReference(100, CANSparkBase.ControlType.kVelocity);
    }
  }


  public void stopMovement(){
    System.out.println("stop movement");
    shooter.setReference(0, CANSparkBase.ControlType.kVelocity);
    loader.setReference(0, CANSparkBase.ControlType.kVelocity);
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
