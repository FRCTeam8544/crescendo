package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStopAuto extends SequentialCommandGroup{

    public IntakeStopAuto(IntakeSubsystem intake){
        addRequirements(intake);
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> intake.stop(), intake),
                new WaitCommand(AutoConstants.waitTime),
                //new IntakeRollersStopAuto(intake),
                new IntakeRetractAuto(intake).withTimeout(AutoConstants.intakeRetractAutoTimeout)
            )
        );
    }
    
}
