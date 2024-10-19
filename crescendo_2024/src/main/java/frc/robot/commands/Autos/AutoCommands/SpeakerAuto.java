package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpeakerAuto extends Command{

    private ShootSubsystem shooter;
    private IntakeSubsystem intake;

    public SpeakerAuto(ShootSubsystem shooter, IntakeSubsystem intake){
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        shooter.shoot(5000);
        if (shooter.atSpeed.getAsBoolean()){
            intake.feedTheMachine();
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted){
            shooter.stop();
            intake.stop();
        }
    }

    @Override
    public boolean isFinished(){
        //return shooter.noteInShooter.getAsBoolean();
        return false;
    }
    
}
