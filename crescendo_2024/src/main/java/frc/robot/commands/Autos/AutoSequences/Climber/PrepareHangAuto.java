package frc.robot.commands.Autos.AutoSequences.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autos.AutoCommands.Climber.ClimbUpAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeExtendAuto;
import frc.robot.subsystems.ClimberElevator;
import frc.robot.subsystems.IntakeSubsystem;

public class PrepareHangAuto extends SequentialCommandGroup{
    
    public PrepareHangAuto(IntakeSubsystem intake, ClimberElevator climber, XboxController controller){
        addCommands(
            new SequentialCommandGroup(
                new IntakeExtendAuto(intake).withTimeout(IntakeConstants.intakeCommandExtendTimeout),
                new ClimbUpAuto(climber, controller)
            )
        );
    }  
}
