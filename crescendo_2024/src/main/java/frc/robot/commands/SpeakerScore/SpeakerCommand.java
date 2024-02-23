package frc.robot.commands.SpeakerScore;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class SpeakerCommand extends Command{
    

    ShootSubsystem shooter;
    XboxController controller;
    IntakeSubsystem intake;
    public SpeakerCommand(ShootSubsystem shooter,IntakeSubsystem intake, XboxController controller){
        this.shooter = shooter;
        this.controller = controller;
        this.intake = intake;
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if (controller.getLeftTriggerAxis() > 0.1){
            shooter.shoot(5000);
        }
        else if (controller.getRightTriggerAxis() > 0.1){
            shooter.shoot(5000);
            intake.feedTheMachine();
        }
    }

    @Override
    public void end(boolean interupted){
        if (interupted){
            shooter.shoot(0);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
