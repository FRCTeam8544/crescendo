package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.ClimbDownAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.IntakeSubsystem;

public class FinishHangAuto extends SequentialCommandGroup{

    public FinishHangAuto(IntakeSubsystem intake, ClimberElevator climber){
        addCommands(
            new ClimbDownAuto(climber).withTimeout(1.5),
            new IntakeRetractAuto(intake).withTimeout(1.75)
        );
    }
    
}
