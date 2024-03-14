package frc.robot.commands.Autos.AutoSequences.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.intakeRollersAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends SequentialCommandGroup{

    public IntakeAuto(IntakeSubsystem intake){
        addCommands(
            new ParallelCommandGroup(
                new intakeRollersAuto(intake),
                new IntakeExtendAuto(intake).withTimeout(IntakeConstants.intakeCommandExtendTimeout)
            ).unless(intake.properNoteInIntake),
            new SequentialCommandGroup(
                new IntakeRetractAuto(intake).withTimeout(IntakeConstants.intakeCommandRetractTimeout)
            ));
    }
}