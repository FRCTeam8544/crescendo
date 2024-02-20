package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeInit extends SequentialCommandGroup{

    
    public IntakeInit(IntakeSubsystem intake){

        new SequentialCommandGroup(
            new RunCommand(() -> intake.moveArm(100), intake).until(intake.atSetpoint)
        );
    }
    
}
