package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class HandoffCommand extends Command{

    IntakeSubsystem intake;
    ShootSubsystem shooter;
    public HandoffCommand(IntakeSubsystem intake, ShootSubsystem shooter){
        this.shooter = shooter;
        this.intake = intake;

    }
    

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (!shooter.noteInShooter.getAsBoolean()){
            intake.rageAgainsTheMachine();
            shooter.handoff();
            intake.moveArm(20);
        }
        else{
            intake.stop();
            shooter.stop(0);
        }
        

    }

    @Override
    public void end(boolean interupted){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
