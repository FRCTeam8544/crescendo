package frc.robot.commands.Autos.AutoCommands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class intakeRollersAuto extends Command{

    IntakeSubsystem intake;
    public intakeRollersAuto(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (intake.noteInside){intake.suckySuck();}
        else{intake.stop();}
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.stop();}
    }

    @Override
    public boolean isFinished(){
        if (!intake.noteInside){
            intake.stop();
            return true;
        }
        return false;
    }
}
