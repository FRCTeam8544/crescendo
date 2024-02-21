package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRun extends SequentialCommandGroup{
    

    public IntakeRun(IntakeSubsystem intake){

        new SequentialCommandGroup(
            //new RunCommand(() -> intake.suckySuck(), intake).until(intake.noteInIntake),
            //new RunCommand(() -> intake.moveArm(0), intake)
        );


    }
}
