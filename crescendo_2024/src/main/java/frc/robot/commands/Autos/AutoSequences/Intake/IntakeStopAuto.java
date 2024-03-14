package frc.robot.commands.Autos.AutoSequences.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeRollersStopAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.intakeRollersAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopAuto extends SequentialCommandGroup{

    public IntakeStopAuto(IntakeSubsystem intake){
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> intake.stop(), intake),
                new IntakeRetractAuto(intake).withTimeout(IntakeConstants.intakeCommandRetractTimeout)
            )
        );
    }
}
