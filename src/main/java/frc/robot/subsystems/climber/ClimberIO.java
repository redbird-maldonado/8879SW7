package frc.robot.subsystems.climber;

// import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  // @AutoLog
  public static class ClimberIOInputs {
    public double climberCurrent = 0.0;
    public double climberVelocity = 0.0;
    public double climberPosition = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
  public default void setClimberVoltage(double voltage) {}
}