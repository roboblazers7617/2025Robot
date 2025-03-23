// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ScoringPoses;
import frc.robot.Constants.DashboardConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LoggingConstants;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.util.Util;
import frc.robot.util.Elastic;
import frc.robot.Constants.OperatorConstants.GamepieceMode;
import frc.robot.Constants.ArmPosition;
import frc.robot.commands.StubbedCommands;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainControls;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeRamp.Ramp;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
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
	/**
	 * The sendable chooser for the autonomous command. This is added in the setAutoChooser method which is run when autobuilder is created after an alliance is selected.
	 */
	private SendableChooser<Command> autoChooser;
	// The robot's subsystems and commands are defined here...
	@NotLogged
	private final Drivetrain drivetrain = new Drivetrain(DrivetrainConstants.CONFIG_DIR);
	@NotLogged
	private final DrivetrainControls drivetrainControls = new DrivetrainControls(drivetrain);
	@NotLogged
	private final Dashboard dashboard = new Dashboard(drivetrain, this);
	private final EndEffector endEffector = new EndEffector(this);
	private final Elevator elevator = new Elevator(this);
	private final Ramp ramp = new Ramp();
	private final Climber climber = new Climber();

	/**
	 * The Controller used by the Driver of the robot, primarily controlling the drivetrain.
	 */
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	/**
	 * The Controller used by the Operator of the robot, primarily controlling the superstructure.
	 */
	@NotLogged
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

		// create named commands for autonomous
		createNamedCommands();
		// Configure the trigger bindings
		configureDriverControls();
		configureOperatorControls();
		// Configure the Limelight mode switching
		// new Trigger(DriverStation::isEnabled).onTrue(drivetrain.getVision().onEnableCommand());
		// new Trigger(DriverStation::isDisabled).onTrue(drivetrain.getVision().onDisableCommand());
		// By default interact with Coral
		gamepieceMode = GamepieceMode.CORAL_MODE;
	}

	/**
	 * This method is run at the start of Auto.
	 */
	public void autoInit() {
		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.AUTO_TAB_NAME);
		}

		// Configure AutoBuilder if not already configured
		Auto.setupPathPlannerFailsafe(drivetrain);

		elevator.elevatorInit();
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Reset the last angle so the robot doesn't try to spin.
		drivetrain.resetLastAngleScalar();

		// Set the Elastic tab
		if (!LoggingConstants.DEBUG_MODE) {
			Elastic.selectTab(DashboardConstants.TELEOP_TAB_NAME);
		}
		// if (StubbedCommands.EndEffector.isHoldingAlage()) {
		// gamepieceMode = GamepieceMode.ALGAE_MODE;
		// }

		// else {
		// gamepieceMode = GamepieceMode.CORAL_MODE;
		// }

		// Configure AutoBuilder if not already configured
		Auto.setupPathPlannerFailsafe(drivetrain);

		elevator.elevatorInit();
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Driver Controller buttons.
	 */
	private void configureDriverControls() {
		// Set the default drivetrain command (used for the driver controller)
		if (RobotBase.isSimulation()) {
			// Heading control
			drivetrain.setDefaultCommand(drivetrainControls.driveFieldOrientedDirectAngleSimCommand(driverController));
		} else {
			// Heading control
			drivetrain.setDefaultCommand(drivetrainControls.driveFieldOrientedDirectAngleCommand(driverController));
			// Angular velocity control
			driverController.leftBumper()
					.whileTrue(drivetrainControls.driveFieldOrientedAngularVelocityCommand(driverController));
		}

		driverController.a().onTrue(ramp.RampDeploy());
		driverController.b().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.TRANSLATION_SCALE_FAST));
		driverController.x().whileTrue(drivetrain.lockCommand());
		driverController.y().onTrue(elevator.SetPositionCommand(ArmPosition.STOW).andThen(ramp.RampRetract()));

		driverController.povUp().whileTrue(climber.RaiseClimber());
		driverController.povDown().whileTrue(climber.LowerClimber());

		// Scoring pose pathfinding
		driverController.leftTrigger()
				.and(isAlgaeModeTrigger)
				.whileTrue(Commands.either(drivetrain.driveToNearestPoseCommand(ScoringPoses.ALGAE_SCORING_POSES_RED), drivetrain.driveToNearestPoseCommand(ScoringPoses.ALGAE_SCORING_POSES_BLUE), () -> Util.isRedAlliance()));
		driverController.leftTrigger()
				.and(isCoralModeTrigger)
				.whileTrue(Commands.either(drivetrain.driveToNearestPoseCommand(ScoringPoses.CORAL_SCORING_POSES_RED_LEFT), drivetrain.driveToNearestPoseCommand(ScoringPoses.CORAL_SCORING_POSES_BLUE_LEFT), () -> Util.isRedAlliance()));
		driverController.rightTrigger()
				.and(isAlgaeModeTrigger)
				.whileTrue(Commands.either(drivetrain.driveToNearestPoseCommand(ScoringPoses.ALGAE_SCORING_POSES_RED), drivetrain.driveToNearestPoseCommand(ScoringPoses.ALGAE_SCORING_POSES_BLUE), () -> Util.isRedAlliance()));
		driverController.rightTrigger()
				.and(isCoralModeTrigger)
				.whileTrue(Commands.either(drivetrain.driveToNearestPoseCommand(ScoringPoses.CORAL_SCORING_POSES_RED_RIGHT), drivetrain.driveToNearestPoseCommand(ScoringPoses.CORAL_SCORING_POSES_BLUE_RIGHT), () -> Util.isRedAlliance()));

		driverController.rightBumper().whileTrue(drivetrainControls.setSpeedMultiplierCommand(() -> DrivetrainConstants.TRANSLATION_SCALE_SLOW));

		driverController.start().onTrue(drivetrain.zeroGyroWithAllianceCommand());
		driverController.back().onTrue(StubbedCommands.Drivetrain.DisableVision());
	}

	/**
	 * Configures {@link Triggers} to bind Commands to the Operator Controller buttons.
	 */
	private void configureOperatorControls() {
		// Set the default elevator command where it moves manually
		elevator.setDefaultCommand(elevator.setSpeedsCommand(() -> MathUtil.applyDeadband(-1.0 * operatorController.getLeftY(), OperatorConstants.DEADBAND), () -> MathUtil.applyDeadband(-1.0 * operatorController.getRightY(), OperatorConstants.DEADBAND)));
		// TODO: #138 Cancel on EndEffector or all mechanism commands?
		operatorController.a()
				.onTrue(endEffector.StopIntakeMotor());
		operatorController.b()
				.or(operatorController.rightTrigger())
				.and(isAlgaeModeTrigger)
				.onTrue(endEffector.AlgaeIntake());
		operatorController.b()
				.or(operatorController.rightTrigger())
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION)
						.andThen(endEffector.CoralIntake())
						// .andThen(endEffector.singleBeamAdjusterCoralIntake())
						.andThen(elevator.SetPositionCommand(ArmPosition.STOW)));
		// .onTrue(elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION))
		// .whileTrue(endEffector.emergencyCoralIntake())
		// .onFalse(elevator.SetPositionCommand(ArmPosition.STOW));
		operatorController.x()
				.and(() -> gamepieceMode == GamepieceMode.ALGAE_MODE) // temp
				.onTrue(elevator.SetPositionCommand(ArmPosition.STOW_ALGAE)
						.alongWith(endEffector.StopIntakeMotor()));
		operatorController.x()
				.and(() -> gamepieceMode == GamepieceMode.CORAL_MODE) // temp
				.onTrue(elevator.SetPositionCommand(ArmPosition.STOW)
						.alongWith(endEffector.StopIntakeMotor()));
		operatorController.y()
				.or(operatorController.leftTrigger())
				.and(isAlgaeModeTrigger)
				.onTrue(endEffector.AlgaeOuttake());
		operatorController.y()
				.or(operatorController.leftTrigger())
				.and(isCoralModeTrigger)
				.whileTrue(endEffector.CoralOuttake())
				// .whileTrue(endEffector.singleBeamAdjusterCoralOuttake())
				// .whileTrue(endEffector.emergencyCoralOuttake())
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4_HIGH).onlyIf(() -> elevator.getElevatorTarget() == ArmPosition.OUTTAKE_CORAL_LEVEL_4.ELEVATOR_POSITION));

		operatorController.povDown()
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_2));
		operatorController.povDown()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_1));
		operatorController.povLeft()
				.or(operatorController.povRight())
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_ALGAE_PROCESSOR));
		operatorController.povLeft()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_2));
		operatorController.povRight()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_3));
		// POV Right Algae mode is handeled above with POV Left
		operatorController.povUp()
				.and(isAlgaeModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_3));
		// If in emergency mode CoralBackup Must be off
		operatorController.povUp()
				.and(isCoralModeTrigger)
				.onTrue(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4).alongWith(endEffector.CoralBackup()));

		// Left Bumper is on an or with the Y button above
		operatorController.rightBumper().onTrue(toggleGamepieceModeCommand());
		operatorController.leftBumper().onTrue(endEffector.CoralBackup());
	}

	/**
	 * Gets the {@link #gamepieceMode}.
	 *
	 * @return
	 *         The current {@link #gamepieceMode}.
	 */
	public GamepieceMode getGamepieceMode() {
		return gamepieceMode;
	}

	/**
	 * Switches the {@link #gamepieceMode} to the next mode.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command toggleGamepieceModeCommand() {
		return Commands.runOnce(() -> {
			switch (gamepieceMode) {
				case CORAL_MODE:
					gamepieceMode = GamepieceMode.ALGAE_MODE;
					break;

				case ALGAE_MODE:
					gamepieceMode = GamepieceMode.CORAL_MODE;
					break;
			}
		});
	}

	public boolean isHoldingAlgae() {
		return endEffector.isHoldingAlgae();
	}

	public boolean isHoldingCoral() {
		return endEffector.isHoldingCoral();
	}

	public void createNamedCommands() {
		NamedCommands.registerCommand("Intake Coral", elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION).andThen(endEffector.CoralIntake()));
		NamedCommands.registerCommand("Intake Algae L2", elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_2).andThen(endEffector.AlgaeIntake()));
		NamedCommands.registerCommand("Intake Algae L3", elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_3).andThen(endEffector.AlgaeIntake()));

		NamedCommands.registerCommand("Elevator L1 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_1));
		NamedCommands.registerCommand("Elevator L2 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_2));
		NamedCommands.registerCommand("Elevator L3 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_3));
		NamedCommands.registerCommand("Elevator L4 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4).alongWith(endEffector.CoralBackup()));
		NamedCommands.registerCommand("Elevator Processor Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_ALGAE_PROCESSOR));

		NamedCommands.registerCommand("Stow Empty", elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION));

		NamedCommands.registerCommand("Score Coral L1-3", endEffector.CoralOuttake());
		NamedCommands.registerCommand("Score Coral L4", endEffector.CoralOuttake().alongWith(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4_HIGH)));
		NamedCommands.registerCommand("Score Algae", endEffector.AlgaeOuttake());
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// resetLastAngleScalar stops the robot from trying to turn back to its original angle after the auto ends
		if (autoChooser == null) {
			return Commands.runOnce(() -> System.out.println("Auto builder not made! Is the alliance set?"));
		}
		return autoChooser.getSelected()
				.finallyDo(drivetrain::resetLastAngleScalar);
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
