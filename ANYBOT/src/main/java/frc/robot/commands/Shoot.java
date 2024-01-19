// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shoot_subsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class Shoot extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final shoot_subsystem shoot_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public Shoot(shoot_subsystem flyingFish) {
    this.shoot_subsystem = flyingFish;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(flyingFish);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(RobotContainer.bingus.getRawButtonPressed(Constants.loadButtoon))
      shoot_subsystem.loadShooter();
    else if(RobotContainer.bingus.getRawButtonReleased(Constants.loadButtoon))
      shoot_subsystem.stopIt();

    if(RobotContainer.bingus.getRawButtonPressed(Constants.shootButton))
      shoot_subsystem.fireInTheHole();
    else if(RobotContainer.bingus.getRawButtonReleased(Constants.shootButton))
      shoot_subsystem.stopIt();

    if(RobotContainer.bingus.getRawButtonPressed(Constants.emergencyStop))
      shoot_subsystem.stopIt();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
