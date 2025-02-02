// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.OperatorConstants.GAMEPIECE_MODE;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.StubbedCommands;
import frc.robot.util.DrivetrainUtil;
import frc.robot.subsystems.Dashboard;
import frc.robot.subsystems.Drivetrain;
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

	// Replace with CommandPS4Controller or CommandJoystick if needed
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	@NotLogged
	private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);
	@NotLogged
	private final Command driveFieldOrientedDirectAngle = drivetrain.driveFieldOrientedCommand(DrivetrainUtil.driveDirectAngle(drivetrain, driverController));
	@NotLogged
	private final Command driveFieldOrientedAnglularVelocity = drivetrain.driveFieldOrientedCommand(DrivetrainUtil.driveAngularVelocity(drivetrain, driverController));
	@NotLogged
	private final Command driveFieldOrientedDirectAngleSim = drivetrain.driveFieldOrientedCommand(DrivetrainUtil.driveDirectAngleSim(drivetrain, driverController));

	private GAMEPIECE_MODE gamepieceMode = GAMEPIECE_MODE.CORAL_MODE;
	private final Trigger isAlgaeMode = new Trigger(() -> (gamepieceMode == GAMEPIECE_MODE.ALGAE_MODE));
	private final Trigger isCoralMode = new Trigger(() -> (gamepieceMode == GAMEPIECE_MODE.CORAL_MODE));

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));

		// Configure the trigger bindings
		configureDriverControls();
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
		// Set the default drivetrain command
		drivetrain.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);

		driverController.a().whileTrue(StubbedCommands.Drivetrain.DriverSlowMode());
		driverController.b().whileTrue(StubbedCommands.Drivetrain.DriverFastMode());
		driverController.x().whileTrue(StubbedCommands.Drivetrain.LockWheels());
		driverController.y().onTrue(StubbedCommands.Climber.StowRamp());

		driverController.povDown().whileTrue(StubbedCommands.Climber.ClimberDown());
		driverController.povRight().whileTrue(StubbedCommands.Climber.RampDown());
		driverController.povLeft().whileTrue(StubbedCommands.Climber.RampUp());
		driverController.povUp().whileTrue(StubbedCommands.Climber.AutoClimb());

		driverController.leftBumper().whileTrue(driveFieldOrientedAnglularVelocity.finallyDo(drivetrain::resetLastAngleScalar));
		driverController.rightBumper().whileTrue(StubbedCommands.Drivetrain.AlignMiddleOfTag());
		driverController.leftTrigger().whileTrue(StubbedCommands.Drivetrain.AlignLeftOfTag());
		driverController.rightTrigger().whileTrue(StubbedCommands.Drivetrain.AlignRightOfTag());

		driverController.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain));
		driverController.back().onTrue(StubbedCommands.Drivetrain.DisableVision());
	}

	private void configureOperatorControls() {
		// Set the default elevator command where it moves manually
		((new StubbedCommands()).new Elevator()).setDefaultCommand(StubbedCommands.Elevator.MoveElevatorAndWristManual(() -> (-1 * operatorController.getLeftX()), () -> (-1 * operatorController.getLeftY())));
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
		operatorController.povRight().or(operatorController.povLeft()).and(isAlgaeMode).onTrue(StubbedCommands.Elevator.MoveProcessor());
		operatorController.povRight().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL2());
		// POV Left Algae mode is handeled above with POV Right
		operatorController.povLeft().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL3());
		operatorController.povUp().and(isAlgaeMode).onTrue(StubbedCommands.Elevator.MoveHighAlgae());
		operatorController.povUp().and(isCoralMode).onTrue(StubbedCommands.Elevator.MoveL4());

		// Left Bumper is on an or with the Y button above
		operatorController.rightBumper().onTrue(Commands.runOnce(() -> {
			gamepieceMode = GAMEPIECE_MODE.ALGAE_MODE;
		}, ((new StubbedCommands()).new Elevator()), (new StubbedCommands()).new EndEffector()));
		// Left Trigger is on an or with the B button above
		operatorController.rightBumper().onTrue(Commands.runOnce(() -> {
			gamepieceMode = GAMEPIECE_MODE.CORAL_MODE;
		}, ((new StubbedCommands()).new Elevator()), (new StubbedCommands()).new EndEffector()));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// resetLastAngleScalar stops the robot from trying to turn back to its original angle after the auto ends
		return autoChooser.getSelected().finallyDo(() -> drivetrain.resetLastAngleScalar());
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

	public void teleopInit() {
		if (StubbedCommands.EndEffector.isHoldingAlage()) {
			gamepieceMode = GAMEPIECE_MODE.ALGAE_MODE;
		}
	}
}
