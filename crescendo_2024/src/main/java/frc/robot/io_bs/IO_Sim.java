package frc.robot.io_bs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IO_Sim {
  private DCMotorSim m_driveSim = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
  private DCMotorSim m_turnSim = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);
}
