package frc.robot.commands.Autos.AutoSequences.Shooting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.Autos.AutoCommands.Drive.DriveAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.Intake.intakeRollersAuto;
import frc.robot.commands.Autos.AutoCommands.Speaker.SpeakerAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class PrimaryAuto extends SequentialCommandGroup{

    public PrimaryAuto(DriveSubsystem driveSubsystem, ShootSubsystem shooter, IntakeSubsystem intake){
        
        Pose2d initPose2d = new Pose2d(-3, 0, new Rotation2d(0));
        Translation2d fristTrans = new Translation2d(-0.5, 0);
        Translation2d secondTrans = new Translation2d(-1, 0);
        Pose2d emoPose2d = new Pose2d(0, 0, new Rotation2d(0));

        Pose2d revPose = new Pose2d(0, 0, new Rotation2d(0));
        Translation2d revTrans = new Translation2d(0.5, 0);
        Translation2d revTransTwo = new Translation2d(1, 0);
        Pose2d revPoseTwo2d = new Pose2d(3, 0, new Rotation2d(0));
        addCommands(
        new SequentialCommandGroup(
            new ParallelCommandGroup(
                new DriveAuto(driveSubsystem, initPose2d, emoPose2d, fristTrans, secondTrans).withTimeout(0.2),
                new SpeakerAuto(shooter, intake).withTimeout(1.5)
            ),
            new ParallelCommandGroup(
                new DriveAuto(driveSubsystem, initPose2d, emoPose2d, fristTrans, secondTrans).withTimeout(0.5),
                new IntakeExtendAuto(intake).withTimeout(IntakeConstants.intakeCommandExtendTimeout),
                new intakeRollersAuto(intake)//.withTimeout(3)
            ),
            
            new ParallelCommandGroup(
                new IntakeRetractAuto(intake).withTimeout(IntakeConstants.intakeCommandRetractTimeout),
                new DriveAuto(driveSubsystem, revPose, revPoseTwo2d, revTrans, revTransTwo)
            ),
            new WaitCommand(0.5),
            new SpeakerAuto(shooter, intake).withTimeout(3)
        ));
    }
}
