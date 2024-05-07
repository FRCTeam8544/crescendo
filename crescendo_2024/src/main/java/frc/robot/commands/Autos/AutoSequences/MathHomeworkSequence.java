package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.MathHomework;
import frc.robot.subsystems.DriveSubsystem;

public class MathHomeworkSequence extends SequentialCommandGroup{

    public MathHomeworkSequence(DriveSubsystem driveSubsystem){
        addCommands(
            new SequentialCommandGroup(
                new MathHomework(driveSubsystem).withTimeout(10)
            )
        );
    }
    
}
