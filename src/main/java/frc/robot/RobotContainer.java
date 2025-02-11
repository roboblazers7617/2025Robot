// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.util.Util;
import frc.robot.subsystems.Dashboard;
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
	private final Drivetrain drivetrain = new Drivetrain(DrivetrainConstants.CONFIG_DIR);
	private final Dashboard dashboard = new Dashboard(drivetrain, this);

	// Replace with CommandPS4Controller or CommandJoystick if needed
	@NotLogged
	private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
	private final ButtonBox buttonBox = new ButtonBox();

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Publish version metadata
		VersionConstants.publishNetworkTables(NetworkTableInstance.getDefault().getTable("/Metadata"));

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * This method is run at the start of Teleop.
	 */
	public void teleopInit() {
		// Reset the last angle so the robot doesn't try to spin.
		drivetrain.resetLastAngleScalarByAlliance();
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
	private void configureBindings() {
		// Set the default drivetrain command (used for the driver controller)
		if (RobotBase.isSimulation()) {
			// Heading control
			drivetrain.setDefaultCommand(drivetrain.driveFieldOrientedDirectAngleSimControllerCommand(driverController));
		} else {
			// ButtonBox heading control
			drivetrain.setDefaultCommand(drivetrain.driveFieldOrientedButtonBoxCommand(buttonBox));
		}
		// TODO: (Max) This lets the driver move to the closest reef tag but how do they make it go to the
		// left or right reef branch of that tag? What if they are on the right side of the tag but
		// want to drive to the left branch?
		// TODO: (Max) Shouldn't this be a whileTrue to allow them to cancel the command if not longer desired?
		driverController.x().onTrue(Commands.either(drivetrain.driveToNearestPoseCommand(FieldConstants.Reef.SCORING_POSES_RED), drivetrain.driveToNearestPoseCommand(FieldConstants.Reef.SCORING_POSES_BLUE), () -> Util.isRedAlliance()));
		// TODO: (Max) How does a driver have it align/drive to the 1) coral station and 2) processor?

		// TODO: transfer to dashboard
		driverController.start().onTrue(Commands.runOnce(() -> drivetrain.zeroGyro(), drivetrain));
		driverController.back().onTrue(drivetrain.centerModulesCommand());
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
