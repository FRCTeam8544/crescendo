package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos.AutoCommands.DriveAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class ShootAuto extends SequentialCommandGroup{
    Pose2d initPose2d = new Pose2d(-3, 0, new Rotation2d(0));
    Translation2d fristTrans = new Translation2d(-0.5, 0);
    Translation2d secondTrans = new Translation2d(-1, 0);
    Pose2d emoPose2d = new Pose2d(0, 0, new Rotation2d(0));

    public ShootAuto(ShootSubsystem shooter, IntakeSubsystem intake, DriveSubsystem drive){
        addCommands(
            new ParallelCommandGroup(
                new DriveAuto(drive, initPose2d, emoPose2d, fristTrans, secondTrans).withTimeout(0.2),
                new SpeakerAuto(shooter, intake).withTimeout(AutoConstants.speakerAutoTimeout)
            )
        );
    }
    
}
