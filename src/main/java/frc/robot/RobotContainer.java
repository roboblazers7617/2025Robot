// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.GAMEPIECE_MODE;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.StubbedCommands;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Util;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

	// Replace with CommandPS4Controller or CommandJoystick if needed
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	@NotLogged
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

	private GAMEPIECE_MODE gamepieceMode;
	private final Trigger isAlgaeMode = new Trigger(() -> (gamepieceMode == GAMEPIECE_MODE.ALGAE_MODE));
	private final Trigger isCoralMode = new Trigger(() -> (gamepieceMode == GAMEPIECE_MODE.CORAL_MODE));

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));

		// Configure the trigger bindings
		configureDriverControls();
		configureOperatorControls();
		// By default interact with Coral
		gamepieceMode = GAMEPIECE_MODE.CORAL_MODE;
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Reset the last angle so the robot doesn't try to spin.
		if (Util.isRedAlliance()) {
			// TODO: (Max) Does this work if you enable/disable as Red alliance multiple times? Won't it keep
			// switing it by 180 degrees each time?
			drivetrain.resetLastAngleScalarInverted();
		} else {
			drivetrain.resetLastAngleScalar();
		}

		if (StubbedCommands.EndEffector.isHoldingAlage()) {
			gamepieceMode = GAMEPIECE_MODE.ALGAE_MODE;
		}
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
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

		// TODO: (Max) This lets the driver move to the closest reef tag but how do they make it go to the
		// left or right reef branch of that tag? What if they are on the right side of the tag but
		// want to drive to the left branch?
		// TODO: (Max) Shouldn't this be a whileTrue to allow them to cancel the command if not longer desired?
		driverController.leftTrigger().onTrue(Commands.either(drivetrain.driveToNearestPoseCommand(FieldConstants.Reef.SCORING_POSES_RED), drivetrain.driveToNearestPoseCommand(FieldConstants.Reef.SCORING_POSES_BLUE), () -> Util.isRedAlliance()));
		// TODO: (Max) How does a driver have it align/drive to the 1) coral station and 2) processor?

		driverController.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain));
		driverController.back().onTrue(StubbedCommands.Drivetrain.DisableVision());
	}

	private void configureOperatorControls() {
		// Set the default elevator command where it moves manually
		/*
		 * StubbedCommands.Elevator elevator = (new StubbedCommands()).new Elevator();
		 * elevator.setDefaultCommand(elevator.MoveElevatorAndWristManual(() -> (-1 * operatorController.getLeftX()), () -> (-1 * operatorController.getLeftY())));
		 */
		// Acts to cancel the currently running command, such as intaking or outaking
		operatorController.a().onTrue(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector())));
		operatorController.b().or(operatorController.leftTrigger()).and(isAlgaeMode).onTrue(StubbedCommands.EndEffector.IntakeAlgae().andThen(StubbedCommands.Elevator.StowAlgae()));
		operatorController.b().or(operatorController.leftTrigger()).and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveIntakeCoral().andThen(StubbedCommands.EndEffector.IntakeCoral(), StubbedCommands.Elevator.StowCoral()));
		operatorController.x().and(isAlgaeMode).onTrue(StubbedCommands.Elevator.StowAlgae().alongWith(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector()))));
		operatorController.x().and(isCoralMode).onTrue(StubbedCommands.Elevator.StowCoral().alongWith(Commands.runOnce((() -> {}), (new StubbedCommands().new EndEffector()))));
		operatorController.y().or(operatorController.leftBumper()).and(isAlgaeMode).onTrue(StubbedCommands.EndEffector.OutakeAlgae());
		operatorController.y().or(operatorController.leftBumper()).and(isCoralMode).onTrue(StubbedCommands.EndEffector.OutakeCoral());

		operatorController.povDown().and(isAlgaeMode).onTrue(StubbedCommands.Elevator.MoveLowAlgae());
		operatorController.povDown().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL1());
		operatorController.povLeft().or(operatorController.povRight()).and(isAlgaeMode).onTrue(StubbedCommands.Elevator.MoveProcessor());
		operatorController.povLeft().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL2());
		operatorController.povRight().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL3());
		// POV Right Algae mode is handeled above with POV Left
		operatorController.povUp().and(isAlgaeMode).onTrue(StubbedCommands.Elevator.MoveHighAlgae());
		operatorController.povUp().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL4());

		// Left Bumper is on an or with the Y button above
		operatorController.rightBumper().onTrue(Commands.runOnce(() -> {
			gamepieceMode = GAMEPIECE_MODE.ALGAE_MODE;
		}));
		// Left Trigger is on an or with the B button above
		operatorController.rightTrigger().onTrue(Commands.runOnce(() -> {
			gamepieceMode = GAMEPIECE_MODE.CORAL_MODE;
		}));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// resetLastAngleScalar stops the robot from trying to turn back to its original angle after the auto ends
		return autoChooser.getSelected();
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
