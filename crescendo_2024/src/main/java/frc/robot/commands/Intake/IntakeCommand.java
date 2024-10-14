package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{
    
    IntakeSubsystem intake;
    XboxController controller;
    public IntakeCommand(IntakeSubsystem intake, XboxController controller){
        this.intake = intake;
        this.controller = controller;
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        if (controller.getLeftTriggerAxis() > 0.1)
            intake.moveArm(0);
    }

    @Override
    public void end(boolean interupted){}

    @Override
    public boolean isFinished(){
        return true;
    }
}
