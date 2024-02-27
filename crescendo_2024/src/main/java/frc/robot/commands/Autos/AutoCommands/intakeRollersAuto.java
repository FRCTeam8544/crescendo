package frc.robot.commands.Autos.AutoCommands;

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
        intake.suckySuck();
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.stop();}
    }

    @Override
    public boolean isFinished(){
        return intake.noteInIntake.getAsBoolean();
    }
    
}