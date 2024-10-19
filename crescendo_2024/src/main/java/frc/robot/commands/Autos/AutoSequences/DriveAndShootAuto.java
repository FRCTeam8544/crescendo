package frc.robot.commands.Autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
                new SpeakerAuto(shootSubsystem, intakeSubsystem).withTimeout(1.5),
                //speakerAuto,
                
                   
                new IntakeExtendAuto(intakeSubsystem).withTimeout(1.25),
                new ParallelCommandGroup(
                    new intakeRollersAuto(intakeSubsystem),
                    new NewDriveAuto(driveSubsystem, 0,-0.5 ,0).withTimeout(1.5)
                ),

                new ParallelCommandGroup(
                    new IntakeRetractAuto(intakeSubsystem).withTimeout(1.75),
                    new NewDriveAuto(driveSubsystem, 0, 0.5, 0).withTimeout(1.5)
                ),
                new WaitCommand(0.5),
                new SpeakerAuto(shootSubsystem, intakeSubsystem).withTimeout(1.5)
                //speakerAuto
            )
        );
    }
    
}
