package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeExtendAuto extends Command{


    IntakeSubsystem intake;
    public IntakeExtendAuto(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        System.out.println("initialize extend");
    }

    @Override
    public void execute(){
        System.out.println("exec Extend lim: "+ intake.reverseLimitSwitch.getAsBoolean());
        if (!intake.reverseLimitSwitch.getAsBoolean()){intake.testRotate(false);}
        else{intake.rotateStop();}
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.rotateStop();}
        System.out.println("extend end");
    }

    @Override
    public boolean isFinished(){
        System.out.println("extend fin");
        return false;
    }
    
}
