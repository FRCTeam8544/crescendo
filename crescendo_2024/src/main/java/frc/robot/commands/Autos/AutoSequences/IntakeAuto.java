package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.intakeRollersAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends SequentialCommandGroup{

    public IntakeAuto(IntakeSubsystem intake){
        addCommands(
            new ParallelCommandGroup(//has issues with rollers
                new IntakeExtendAuto(intake).withTimeout(1),
                new intakeRollersAuto(intake)
            ).unless(intake.properNoteInIntake),
            new SequentialCommandGroup(
                new IntakeRetractAuto(intake).withTimeout(1.75)
            ));
    }

}