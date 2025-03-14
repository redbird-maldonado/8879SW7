package frc.robot.subsystems.climber;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

//
public class ClimberIOSparkMax implements ClimberIO {
    SparkMax climberMotor;
    RelativeEncoder climberEncoder;

    public ClimberIOSparkMax() {
        climberMotor = new SparkMax(22, MotorType.kBrushless);
        climberEncoder = climberMotor.getEncoder();

        // CONFIG CLIMBER MOTOR
        SparkMaxConfig configClimber = new SparkMaxConfig();
        configClimber
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
        configClimber.encoder
                .positionConversionFactor(1000)
                .velocityConversionFactor(1000);
        
        climberMotor.configure(configClimber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // @Override
    // public void updateInputs(IntakeIOInputs inputs) {
    // inputs.coralWristCurrent = coralWrist.getOutputCurrent();
    // inputs.coralWristVelocity = coralWrist.getEncoder().getVelocity();
    // inputs.coralWristPosition = coralWrist.getEncoder().getPosition();
    // }

    @Override
    public void setClimberVoltage(double voltage) {
        climberMotor.setVoltage(voltage);
    }

    // @Override
    public void stop() {
        climberMotor.setVoltage(0);
    }
}
