// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;

public class ShootSubsystem extends SubsystemBase {

  private static CANSparkMax topMotor = new CANSparkMax(Constants.ShooterConstants.TopMotorCANID, CANSparkLowLevel.MotorType.kBrushless);
  private static CANSparkMax bottomMotor = new CANSparkMax(Constants.ShooterConstants.BottomMotorCANID, CANSparkLowLevel.MotorType.kBrushless);


  public ShootSubsystem() {

    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

  }

  public void loadShooter() {
    topMotor.set(-.3);
    bottomMotor.set(-.3);
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
