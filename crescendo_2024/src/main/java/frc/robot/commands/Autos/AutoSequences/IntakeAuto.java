package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ControllerVibrate;
import frc.robot.commands.Autos.AutoCommands.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.intakeRollersAuto;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends SequentialCommandGroup{

    public IntakeAuto(IntakeSubsystem intake, XboxController juliet, XboxController romeo){
        addCommands(
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                    new intakeRollersAuto(intake),
                    new ControllerVibrate(juliet, romeo).withTimeout(0.2)
                ),
                new IntakeExtendAuto(intake).withTimeout(1)
            ).unless(intake.properNoteInIntake),
            new SequentialCommandGroup(
                //new intakeRollersAuto(intake).withTimeout(2),
                new IntakeRetractAuto(intake).withTimeout(1)
                //new ControllerVibrate(juliet, null)
            ));
    }

}