package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class FixedShoot extends SequentialCommandGroup{

    public FixedShoot(ShootSubsystem shooter, IntakeSubsystem intake){
        addCommands(
            new SequentialCommandGroup(
                new SpeakerAuto(shooter, intake).withTimeout(1.5)
            )
        );
    }
    
}
