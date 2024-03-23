package frc.robot.commands.Autos.AutoCommands;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeRollersAuto extends Command{

    IntakeSubsystem intake;
    int count = 0;
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
            count = 0;
        }
        else{
            count = count + 1;//not having ++ or += is so lame
            if (count > 5);
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.stop();}
    }

    @Override
    public boolean isFinished(){
        if (!intake.noteInside && count > 5){
            intake.stop();
            return true;
        }
        return false;
    }
    
}
