package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Autos.AutoCommands.IntakeExtendAuto;
import frc.robot.commands.Autos.AutoCommands.IntakeRetractAuto;
import frc.robot.commands.Autos.AutoCommands.NewDriveAuto;
import frc.robot.commands.Autos.AutoCommands.SpeakerAuto;
import frc.robot.commands.Autos.AutoCommands.intakeRollersAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class DriveAndShootAuto extends SequentialCommandGroup{

    public DriveAndShootAuto(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShootSubsystem shootSubsystem){//SpeakerAuto speakerAuto){//
        
        addCommands(
            new SequentialCommandGroup(
                new SpeakerAuto(shootSubsystem, intakeSubsystem).withTimeout(AutoConstants.speakerAutoTimeout),
                new IntakeExtendAuto(intakeSubsystem).withTimeout(AutoConstants.intakeExtendAutoTimeout),
                new ParallelCommandGroup(
                    new intakeRollersAuto(intakeSubsystem),
                    new NewDriveAuto(driveSubsystem, 0,-0.5 ,0).withTimeout(AutoConstants.newDriveAutoTimeout)
                ),
                new ParallelCommandGroup(
                    new IntakeRetractAuto(intakeSubsystem).withTimeout(AutoConstants.intakeRetractAutoTimeout),
                    new NewDriveAuto(driveSubsystem, 0, 0.5, 0).withTimeout(AutoConstants.newDriveAutoTimeout)
                ),
                new WaitCommand(AutoConstants.waitTime),
                new SpeakerAuto(shootSubsystem, intakeSubsystem).withTimeout(AutoConstants.speakerAutoTimeout)
            )
        );
    }
    
}
