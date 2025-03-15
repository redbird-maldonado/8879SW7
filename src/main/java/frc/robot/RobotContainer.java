package frc.robot;

import static edu.wpi.first.units.Units.*;

// import java.util.jar.Attributes.Name;

import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.elevator.elevator;
import frc.robot.subsystems.elevator.elevatorIOSparkMax;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
	private final Climber climber;
	private final elevator elevator;
	private final Intake intake;
	private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
	private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
																						// max angular V
	// NORMAL DRIVE - DRIVER STICKS
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 5% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

	// 2025 DRIVER DECLINES THE FOLLOWING
	// private final SwerveRequest.SwerveDriveBrake brake = new
	// SwerveRequest.SwerveDriveBrake();
	// private final SwerveRequest.PointWheelsAt point = new
	// SwerveRequest.PointWheelsAt();

	// PRECISION DRIVE
	private final SwerveRequest.FieldCentric slowMode = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final Telemetry logger = new Telemetry(MaxSpeed);
	private final CommandXboxController driverController = new CommandXboxController(0);
	private final CommandXboxController operatorController = new CommandXboxController(1);
	public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

	// PATH FOLLOWER
	private final SendableChooser<Command> autoChooser;

	public RobotContainer() {

		climber = new Climber(new ClimberIOSparkMax());
		elevator = new elevator(new elevatorIOSparkMax());
		intake = new Intake(new IntakeIOSparkMax()); // INTAKE HAS BOTH WRIST AND ALGAE

		NamedCommands.registerCommand("score", Commands.runOnce(() -> {
			intake.autoSetCoralIntakeVoltage(6); // POSITIVE TO EJECT
		}));

		// new EventTrigger("score").onTrue(Commands.runOnce(() -> {
		// 	System.out.println("I have scored");
		// }));

		NamedCommands.registerCommand("stopScore", Commands.runOnce(() -> {
			System.out.println("REGISTERING STOPSCORE COMMAND");
		}));

		new EventTrigger("stopScore").onTrue(Commands.runOnce(() -> {
			System.out.println("I'm done scoring");
			intake.autoSetCoralIntakeVoltage(0); // POSITIVE TO EJECT
		}));

		autoChooser = AutoBuilder.buildAutoChooser("Tests");
		SmartDashboard.putData("Auto Mode", autoChooser);

		configureBindings();
	}

	private void configureBindings() {
		// THE JOYSTICK CONTROLS ARE HERE: https://tinyurl.com/25xw7zu7
		// NB: X is defined as forward and Y is defined as to the left, according to
		// WPILib convention

		// DEFAULT MODE
		drivetrain.setDefaultCommand(
				// Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-driverController.getLeftY() * MaxSpeed)
						// Drive forward with negative Y (forward)
						.withVelocityY(-driverController.getLeftX() * MaxSpeed)
						// Drive left with negative X (left)
						.withRotationalRate(-driverController.getRightX() * MaxAngularRate)
				// Drive counterclockwise with negative X (left)
				));

		// SLOW MODE
		driverController.rightBumper()
				.whileTrue(drivetrain.applyRequest(() -> slowMode
						.withVelocityX(-driverController.getLeftY() * MaxSpeed * 0.1)
						// Drive forward with negative Y (forward)
						.withVelocityY(-driverController.getLeftX() * MaxSpeed * 0.1)
						// Drive left with negative X (left)
						.withRotationalRate(-driverController.getRightX() * MaxAngularRate * 0.3)));
		// Drive counterclockwise with negative X (left)

		// DRIVER BRAKE
		// driverController.b().whileTrue(drivetrain.applyRequest(() -> brake));
		// driverController.leftTrigger().whileTrue(drivetrain.applyRequest(
		// () -> point.withModuleDirection(new Rotation2d(
		// -driverController.getLeftY(),
		// -driverController.getLeftX()))));

		// Run SysId routines when holding back/start and X/Y.
		// Note that each routine should be run exactly once in a single log.
		// driverController.back().and(driverController.y())
		// .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
		// driverController.back().and(driverController.x())
		// .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
		// driverController.start().and(driverController.y())
		// .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
		// driverController.start().and(driverController.x())
		// .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

		// reset the field-centric heading on left bumper press
		driverController.leftBumper().and(driverController.y()).and(driverController.rightBumper())
				.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

		drivetrain.registerTelemetry(logger::telemeterize);

		/* ELEVATOR STATES */

		/*
		 * LISTEN FOR OVERCURRENT ON CAN - WHAT IS THE MESSAGE FROM THE MOTOR IF IT'S
		 * STALLING
		 */
		/* STOP THE INTAKE */
		/* .andThen(run the intake) *********** time the eject */
		/* MOVE THE COMMANDS ABOVE THE ELVATOR STATES IF WE USE THE ANDTHEN */

		/* SOURCE COMMANDS */
		Command liftToSourceCommand = new RunCommand(() -> elevator.setPosition(Constants.SOURCE_HEIGHT), elevator);
		Command wristToSourceCommand = new RunCommand(() -> intake.wristAngle(Constants.SOURCE_ANGLE), intake);
		ParallelCommandGroup sourceCommandGroup = new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);
		driverController.leftBumper().onTrue(sourceCommandGroup);

		// ZERO THE WRIST
		Command wristZeroCommand = new RunCommand(() -> intake.wristAngle(Constants.ZERO_ANGLE), intake);
		driverController.x().onTrue(wristZeroCommand);

		// ELEVATOR MANUAL UP-DOWN
		Command manualEleUpCommand = new StartEndCommand(() -> elevator.setVoltage(.05),
				() -> elevator.setVoltage(0), elevator);
		operatorController.povUp().whileTrue(manualEleUpCommand);
		Command manualEleDownCommand = new StartEndCommand(() -> elevator.setVoltage(-.05),
				() -> elevator.setVoltage(0), elevator);
		operatorController.povDown().whileTrue(manualEleDownCommand);

		// L0 state
		Command liftToL0CommandEle = new RunCommand(() -> elevator.setPosition(Constants.L0_HEIGHT), elevator);
		Command liftToL0Command = new RunCommand(() -> elevator.setPosition(Constants.L0_HEIGHT), elevator);
		Command wristToL0Command = new RunCommand(() -> intake.wristAngle(Constants.L0_ANGLE), intake);
		ParallelCommandGroup l0CommandGroup = new ParallelCommandGroup(liftToL0Command, wristToL0Command);
		operatorController.a().onTrue(l0CommandGroup);
		driverController.a().onTrue(liftToL0CommandEle); // added L0 to driver

		// L1 state
		Command liftToL1Command = new RunCommand(() -> elevator.setPosition(Constants.L1_HEIGHT), elevator);
		Command wristToL1Command = new RunCommand(() -> intake.wristAngle(Constants.L1_ANGLE), intake);
		ParallelCommandGroup l1CommandGroup = new ParallelCommandGroup(liftToL1Command, wristToL1Command);
		operatorController.b().onTrue(l1CommandGroup);

		// L2 state
		Command liftToL2Command = new RunCommand(() -> elevator.setPosition(Constants.L2_HEIGHT), elevator);
		Command wristToL2Command = new RunCommand(() -> intake.wristAngle(Constants.L2_ANGLE), intake);
		ParallelCommandGroup l2CommandGroup = new ParallelCommandGroup(liftToL2Command, wristToL2Command);
		operatorController.x().onTrue(l2CommandGroup);

		// L3 state
		Command liftToL3Command = new RunCommand(() -> elevator.setPosition(Constants.L3_HEIGHT), elevator);
		Command wristToL3Command = new RunCommand(() -> intake.wristAngle(Constants.L3_ANGLE), intake);
		ParallelCommandGroup l3CommandGroup = new ParallelCommandGroup(liftToL3Command, wristToL3Command);
		operatorController.y().onTrue(l3CommandGroup);

		// KILL RUMBLE
		// operatorController.setRumble(null, MaxAngularRate);

		// ALGAE L-1 state
		Command algae_lift_1 = new RunCommand(() -> elevator.setPosition(Constants.L1_HEIGHT_ALGAE), elevator);
		driverController.leftTrigger().onTrue(algae_lift_1);

		// ALGAE L-2 state
		Command algae_lift_2 = new RunCommand(() -> elevator.setPosition(Constants.L2_HEIGHT_ALGAE), elevator);
		driverController.rightTrigger().onTrue(algae_lift_2);

		/* CLIMBER COMMANDS */
		Command climberUpCommand = new StartEndCommand(() -> climber.setClimberVoltage(-12),
				() -> climber.setClimberVoltage(0), climber);
		driverController.y().whileTrue(climberUpCommand);
		Command climberDownCommand = new StartEndCommand(() -> climber.setClimberVoltage(12),
				() -> climber.setClimberVoltage(0), climber);
		driverController.b().whileTrue(climberDownCommand);

		// NICE TO HAVE - A COMMAND THAT PICKS UP BOTH THE ALGAE AND CORAL

		/* ALGAE COMMANDS */
		Command intakeAlgaeCommand = new StartEndCommand(() -> intake.setAlgaeVoltage(-12),
				() -> intake.setAlgaeVoltage(0),
				intake);
		operatorController.rightBumper().whileTrue(intakeAlgaeCommand);

		Command ejectAlgaeCommand = new StartEndCommand(() -> intake.setAlgaeVoltage(12),
				() -> intake.setAlgaeVoltage(0),
				intake);
		operatorController.rightTrigger().whileTrue(ejectAlgaeCommand);

		/* CORAL COMMANDS */
		Command intakeCoralCommand = new StartEndCommand(() -> intake.setCoralIntakeVoltage(-6),
				() -> intake.setCoralIntakeVoltage(0), intake);
		operatorController.leftBumper().whileTrue(intakeCoralCommand);

		Command ejectCoralCommand = new StartEndCommand(() -> intake.setCoralIntakeVoltage(6),
				() -> intake.setCoralIntakeVoltage(0), intake);
		operatorController.leftTrigger().whileTrue(ejectCoralCommand);

				
		/* PROCESSOR COMMANDS */
		// Command liftToProcessorCommand = new RunCommand(() ->
		// elevator.setPosition(Constants.PROCESSOR_HEIGHT), elevator);

		// Command wristToProcessorCommand = new RunCommand(() ->
		// intake.wristAngle(Constants.PROCESSOR_ANGLE), intake);
		// ParallelCommandGroup processorCommandGroup = new
		// ParallelCommandGroup(liftToProcessorCommand,
		// wristToProcessorCommand);
		// driverController.rightBumper().onTrue(processorCommandGroup);

		// POV (D-PAD) IS ANALOG. NEED DIGITAL TO RUN ONCE .: DON'T USE B/C WON'T
		// TRIGGER
		// driverController.povUp().onTrue(climbCommandGroup);
		// driverController.povDown().onTrue(climbCommandGroup);
		// driverController.povLeft().onTrue(climbCommandGroup);
		// driverController.povRight().onTrue(climbCommandGroup);
	}

	/* AUTONOMY */
	public Command getAutonomousCommand() {
		/* Run the path selected from the auto chooser */
		return autoChooser.getSelected();
	}
}
