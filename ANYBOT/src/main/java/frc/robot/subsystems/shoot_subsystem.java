// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;

public class shoot_subsystem extends SubsystemBase {

  static CANSparkMax motor1 = new CANSparkMax(Constants.Motor1CANID, CANSparkLowLevel.MotorType.kBrushless);
  static CANSparkMax motor2 = new CANSparkMax(Constants.Motor2CANID, CANSparkLowLevel.MotorType.kBrushless);
    


  public shoot_subsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public void loadShooter() {
    motor1.set(-.5);
    motor2.set(-.5);
  }

  public void fireInTheHole(){
    motor1.set(.5);
    motor2.set(.5);
    //this may or may not do smth funky idk
  }

  public void stopIt()
  {
    motor1.set(0);
    motor2.set(0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
