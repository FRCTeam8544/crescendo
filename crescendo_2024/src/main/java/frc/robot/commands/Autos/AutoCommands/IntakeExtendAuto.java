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

    }

    @Override
    public void execute(){
        if (!intake.forwardLimitSwitch.getAsBoolean()){intake.testRotate(true);}
        else{intake.rotateStop();}
    }

    @Override
    public void end(boolean interupted){
        if (interupted){intake.rotateStop();}
    }

    @Override
    public boolean isFinished(){
        return intake.forwardLimitSwitch.getAsBoolean();
    }
    
}