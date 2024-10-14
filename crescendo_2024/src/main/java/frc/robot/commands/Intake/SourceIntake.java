package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SourceIntake extends Command{
    
    IntakeSubsystem intake;
    ShootSubsystem shooter;

    public SourceIntake(IntakeSubsystem intake, ShootSubsystem shooter){
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        intake.sourceIntake();
        shooter.sourceIntake();
    }

    @Override
    public void end(boolean interupted){
        intake.stop();
        shooter.stop();
    }

    @Override
    public boolean isFinished(){
        return false;
    }

}
