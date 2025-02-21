// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.GamepieceMode;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.StubbedCommands;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
	private SendableChooser<Command> autoChooser = new SendableChooser<>();
	// The robot's subsystems and commands are defined here...
	@NotLogged
	private final Drivetrain drivetrain = new Drivetrain(DrivetrainConstants.CONFIG_DIR);
	@NotLogged
	private final Dashboard dashboard = new Dashboard(drivetrain, this);
	@NotLogged
	private final Elevator elevator = new Elevator();

	// Replace with CommandPS4Controller or CommandJoystick if needed
	@NotLogged
	/**
	 * The Controller used by the Driver of the robot, primarily controlling the drivetrain.
	 */
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	@NotLogged
	/**
	 * The Controller used by the Operator of the robot, primarily controlling the superstructure.
	 */
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

	/**
	 * Used to store what the currently select game piece to interact with is.
	 */
	private GamepieceMode gamepieceMode;
	private final Trigger isAlgaeModeTrigger = new Trigger(() -> (gamepieceMode == GamepieceMode.ALGAE_MODE));
	private final Trigger isCoralModeTrigger = new Trigger(() -> (gamepieceMode == GamepieceMode.CORAL_MODE));

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));

		// Configure the trigger bindings
		configureDriverControls();
		configureOperatorControls();
		// By default interact with Coral
		gamepieceMode = GamepieceMode.CORAL_MODE;
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Reset the last angle so the robot doesn't try to spin.
		drivetrain.resetLastAngleScalarByAlliance();

		if (StubbedCommands.EndEffector.isHoldingAlage()) {
			gamepieceMode = GamepieceMode.ALGAE_MODE;
		}

		else {
			gamepieceMode = GamepieceMode.CORAL_MODE;
		}
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Driver Controller buttons.
	 */
	private void configureDriverControls() {
		// Set the default drivetrain command (used for the driver controller)
		if (RobotBase.isSimulation()) {
			// Heading control
			drivetrain.setDefaultCommand(drivetrain.driveFieldOrientedDirectAngleSimControllerCommand(driverController));
		} else {
			// Heading control
			drivetrain.setDefaultCommand(drivetrain.driveFieldOrientedDirectAngleControllerCommand(driverController));
			// Angular velocity control
			driverController.leftBumper()
					.whileTrue(drivetrain.driveFieldOrientedAngularVelocityControllerCommand(driverController));
		}

		driverController.a().whileTrue(StubbedCommands.Drivetrain.DriverSlowMode());
		driverController.b().whileTrue(StubbedCommands.Drivetrain.DriverFastMode());
		driverController.x().whileTrue(StubbedCommands.Drivetrain.LockWheels());
		driverController.y().onTrue(StubbedCommands.Climber.StowRamp());

		driverController.povDown().whileTrue(StubbedCommands.Climber.ClimberDown());
		driverController.povLeft().whileTrue(StubbedCommands.Climber.RampUp());
		driverController.povRight().whileTrue(StubbedCommands.Climber.RampDown());
		driverController.povUp().whileTrue(StubbedCommands.Climber.AutoClimb());

		driverController.rightBumper().whileTrue(StubbedCommands.Drivetrain.AlignMiddleOfTag());
		driverController.leftTrigger().whileTrue(StubbedCommands.Drivetrain.AlignLeftOfTag());
		driverController.rightTrigger().whileTrue(StubbedCommands.Drivetrain.AlignRightOfTag());

		driverController.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain));
		driverController.back().onTrue(StubbedCommands.Drivetrain.DisableVision());
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Operator Controller buttons.
	 */
	private void configureOperatorControls() {
		// Set the default elevator command where it moves manually
		/*
		 * StubbedCommands.Elevator elevator = (new StubbedCommands()).new Elevator();
		 * elevator.setDefaultCommand(elevator.MoveElevatorAndWristManual(() -> (-1 * operatorController.getLeftX()), () -> (-1 * operatorController.getLeftY())));
		 */
		// Acts to cancel the currently running command, such as intaking or outaking
		operatorController.a()
				.onTrue(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector())));
		operatorController.b()
				.or(operatorController.leftTrigger())
				.and(isAlgaeModeTrigger)
				.onTrue(StubbedCommands.EndEffector.IntakeAlgae()
						.andThen(elevator.SetPosition(ArmPosition.STOW_ALGAE)));
		operatorController.b()
				.or(operatorController.leftTrigger())
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.INTAKE_CORAL_CORAL_STATION)
						.andThen(StubbedCommands.EndEffector.IntakeCoral())
						.andThen(elevator.SetPosition(ArmPosition.STOW_CORAL)));
		operatorController.x()
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.STOW_ALGAE)
						.alongWith(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector()))));
		operatorController.x()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.STOW_CORAL)
						.alongWith(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector()))));
		operatorController.y()
				.or(operatorController.leftBumper())
				.and(isAlgaeModeTrigger)
				.onTrue(StubbedCommands.EndEffector.OutakeAlgae());
		operatorController.y()
				.or(operatorController.leftBumper())
				.and(isCoralModeTrigger)
				.onTrue(StubbedCommands.EndEffector.OutakeCoral());

		operatorController.povDown()
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.INTAKE_ALGAE_LEVEL_2));
		operatorController.povDown()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.OUTTAKE_CORAL_LEVEL_1));
		operatorController.povLeft()
				.or(operatorController.povRight())
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.OUTTAKE_ALGAE_PROCESSOR));
		operatorController.povLeft()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.OUTTAKE_CORAL_LEVEL_2));
		operatorController.povRight()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.OUTTAKE_CORAL_LEVEL_3));
		// POV Right Algae mode is handeled above with POV Left
		operatorController.povUp()
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.INTAKE_ALGAE_LEVEL_3));
		operatorController.povUp()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPosition(ArmPosition.OUTTAKE_CORAL_LEVEL_4));

		// Left Bumper is on an or with the Y button above
		operatorController.rightBumper().onTrue(setGamepieceModeCommand(GamepieceMode.ALGAE_MODE));
		// Left Trigger is on an or with the B button above
		operatorController.rightTrigger().onTrue(setGamepieceModeCommand(GamepieceMode.CORAL_MODE));
	}

	public GamepieceMode getGamepieceMode() {
		return gamepieceMode;
	}

	private Command setGamepieceModeCommand(GamepieceMode mode) {
		return Commands.runOnce(() -> {
			gamepieceMode = mode;
		});
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// resetLastAngleScalar stops the robot from trying to turn back to its original angle after the auto ends
		return autoChooser.getSelected()
				.finallyDo(drivetrain::resetLastAngleScalarByAlliance);
	}

	/**
	 * Set the auto chooser
	 *
	 * @param auto
	 *            a sendable chooser with Commands for the autos
	 */
	public void setAutoChooser(SendableChooser<Command> auto) {
		autoChooser = auto;
	}
}
