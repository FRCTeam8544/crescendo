package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.DriveAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootAndMove extends SequentialCommandGroup{

    public ShootAndMove(DriveSubsystem drive, IntakeSubsystem intake, ShootSubsystem shoot){
        Pose2d initPose2d = new Pose2d(-3, 0, new Rotation2d(0));
        Translation2d firstTrans = new Translation2d(-0.5, 0);
        Translation2d secondTrans = new Translation2d(-1, 0);
        Pose2d emoPose2d = new Pose2d(0, 0, new Rotation2d(0));

        addCommands(
            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    new DriveAuto(drive, initPose2d, emoPose2d, firstTrans, secondTrans).withTimeout(0.2),
                    new SpeakerAuto(shoot, intake).withTimeout(1.5)
                ),
                
                new DriveAuto(drive, initPose2d, emoPose2d, firstTrans, secondTrans).withTimeout(0.5)
            )
        );
        
    }
    
}
