package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRollersStopAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRollersAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopAuto extends SequentialCommandGroup{

    public IntakeStopAuto(IntakeSubsystem intake){
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> intake.stop(), intake),
                new IntakeRetractAuto(intake).withTimeout(1)
            )
        );
    }
    
}
