package frc.robot.commands.AmpScore;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShootSubsystem;

public class HandoffCommand extends Command{

    IntakeSubsystem intake;
    ShootSubsystem shooter;
    double count = 0;
    XboxController controller;
    public HandoffCommand(IntakeSubsystem intake, ShootSubsystem shooter, XboxController controller){
        this.shooter = shooter;
        this.intake = intake;
        this.controller = controller;

    }
    

    @Override
    public void initialize(){
        shooter.setBrake();
        System.out.println("handOff Started");
        count = 0;
    }

    @Override
    public void execute(){
        SmartDashboard.putNumber("count", count);
        if (!shooter.noteInShooter.getAsBoolean()){
            intake.rageAgainsTheMachine();
            shooter.handoff();
            //count = 0;
        }
        else if (count < 7){
            //intake.stop();
            count = count + 1;
            //shooter.stop();
        }else{
            //shooter.stop();
            intake.stop();
        }
        

    }

    @Override
    public void end(boolean interupted){
        intake.stop();
        //shooter.stop();
        shooter.setZero();
    }

    @Override
    public boolean isFinished(){
        if ((shooter.noteInShooter.getAsBoolean() && count >= 7) || !controller.getLeftBumper()){
            System.out.println("handOffEnded");
            return true;
        }
        //return shooter.noteInShooter.getAsBoolean() && count > 6;
        return false;
    }
}
