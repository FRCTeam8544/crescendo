package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollersAuto extends Command{

    IntakeSubsystem intake;
    //int count = 0;
    public intakeRollersAuto(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (intake.noteInside){
            intake.suckySuck();
            //count = 0;
        }
        else{
            //if (count > 20){intake.stop();}
            intake.stop();
            //count = count + 1;
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.stop();}
    }

    @Override
    public boolean isFinished(){
        if (!intake.noteInside){ //&& count > 19){
            intake.stop();
            return true;
        }
        return false;
    }
    
}
