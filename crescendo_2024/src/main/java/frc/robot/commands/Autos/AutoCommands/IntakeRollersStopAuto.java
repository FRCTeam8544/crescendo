package frc.robot.commands.Autos.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeRollersStopAuto extends Command{

    IntakeSubsystem intake;
    public IntakeRollersStopAuto(IntakeSubsystem intake){
        this.intake = intake;
    }

    @Override
    public void initialize(){
        intake.stop();
    }
    @Override
    public void execute(){}

    @Override
    public void end(boolean Interupted){}

    @Override
    public boolean isFinished(){
        intake.stop();
        return true;
    }
}
