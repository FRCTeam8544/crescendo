package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRetractAuto extends Command{

    IntakeSubsystem intake;
    public IntakeRetractAuto(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        System.out.println("intake retract");
    }

    @Override
    public void execute(){
        intake.testRotate(true);
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.rotateStop();}
    }   
    
    @Override
    public boolean isFinished(){
        return false;
    }
}
