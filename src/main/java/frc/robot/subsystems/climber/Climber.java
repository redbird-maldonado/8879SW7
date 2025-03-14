package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  ClimberIO io;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  public void setClimberVoltage(double voltage) {
    io.setClimberVoltage(voltage);
  }

}
