package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.ClimbUpAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeExtendAuto;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.IntakeSubsystem;

public class PrepareHangAuto extends SequentialCommandGroup{
    
    public PrepareHangAuto(IntakeSubsystem intake, ClimberElevator climber, XboxController controller){
        addCommands(
            new SequentialCommandGroup(
                new IntakeExtendAuto(intake).withTimeout(1),
                new ClimbUpAuto(climber, controller)
            )
        );
    }
    
}
