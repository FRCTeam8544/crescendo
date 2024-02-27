package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.AutoCommands.DriveAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class testAuto extends SequentialCommandGroup{

    public testAuto(DriveSubsystem driveSubsystem, ShootSubsystem shooter, IntakeSubsystem intake){
        
        Pose2d initPose2d = new Pose2d(0, 0, new Rotation2d(3));
        Translation2d fristTrans = new Translation2d(0.5, 0);
        Translation2d secondTrans = new Translation2d(1, 0);
        Pose2d emoPose2d = new Pose2d(1.5, 0, new Rotation2d(3));
        addCommands(
        new SequentialCommandGroup(
            new SpeakerAuto(shooter, intake).withTimeout(1.5),
            new DriveAuto(driveSubsystem, initPose2d, emoPose2d, fristTrans, secondTrans)

        ));
    }
}
