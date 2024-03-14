package frc.robot.commands.Autos.AutoSequences.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.Climber.ClimbDownAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeRetractAuto;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.IntakeSubsystem;

public class FinishHangAuto extends SequentialCommandGroup{

    public FinishHangAuto(IntakeSubsystem intake, ClimberElevator climber){
        addCommands(
            new ClimbDownAuto(climber),
            new IntakeRetractAuto(intake)
        );
    }
}
