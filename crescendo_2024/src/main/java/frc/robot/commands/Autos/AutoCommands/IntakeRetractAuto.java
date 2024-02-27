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

    }

    @Override
    public void execute(){
        intake.testRotate(false);
    }

    @Override
    public void end(boolean interupted){
        intake.rotateStop();
    }   
    
    @Override
    public boolean isFinished(){
        return intake.reverseLimitSwitch.getAsBoolean();
    }
}
