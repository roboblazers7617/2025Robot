// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants.GamepieceMode;
import frc.robot.commands.RunOnceDeferred;
import frc.robot.commands.drivetrain.LockWheelsCommand;
import frc.robot.subsystems.Auto;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * A class that sets up the driverstation dashboard for the robot.
 */

public class Dashboard {
	final Drivetrain drivetrain;
	final RobotContainer robotContainer;
	final SendableChooser<DriverStation.Alliance> alliancePicker;
	final SendableChooser<Pose2d> pose;
	SendableChooser<Command> auto;
	/** Is the robot in coral mode, if not then it is assumed it is in algae mode. */
	private final BooleanPublisher isCoralMode;

	/** Creates a new Dashboard. */
	public Dashboard(Drivetrain drivetrain, RobotContainer robotContainer) {
		this.drivetrain = drivetrain;
		this.robotContainer = robotContainer;

		alliancePicker = new SendableChooser<DriverStation.Alliance>();
		alliancePicker.setDefaultOption("None", null);
		alliancePicker.addOption("Blue", DriverStation.Alliance.Blue);
		alliancePicker.addOption("Red", DriverStation.Alliance.Red);

		// TODO: #103 (Brandon) What happens if you change the color multiple times? Does this work or crash?
		alliancePicker.onChange((alliance) -> {
			new RunOnceDeferred(() -> {
				configureAutoBuilder(alliance);
			}).ignoringDisable(true).schedule();
		});

		pose = new SendableChooser<Pose2d>();
		pose.setDefaultOption("center edge on blue side", new Pose2d(.5, 4, new Rotation2d(0)));
		pose.addOption("position 2", new Pose2d(0, 0, new Rotation2d(45)));
		pose.addOption("center edge on red side", new Pose2d(17, 4, new Rotation2d(0)));

		isCoralMode = NetworkTableInstance.getDefault().getTable("").getBooleanTopic("CoralMode").publish();
		isCoralMode.set(true);
		new Trigger(() -> robotContainer.getGamepieceMode() == GamepieceMode.CORAL_MODE).onChange(new InstantCommand(() -> isCoralMode.set(robotContainer.getGamepieceMode() == GamepieceMode.CORAL_MODE)));

		// pose.onChange((pose) -> {
		// drivetrain.resetOdometry(pose);
		// });

		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("selected").setString("None");
		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("active").setString("None");
		SmartDashboard.putData("Alliance", alliancePicker);
		SmartDashboard.putData("Pose", pose);
		SmartDashboard.putData("Reset pose to selected position", resetPose());
		SmartDashboard.putData("Toggle brake modes", robotContainer.toggleBrakeModesCommand());
	}

	/**
	 * Configures the auto builder using the drivetrain subsystem. Also sets up the auto chooser on the dashboard.
	 *
	 * @param alliance
	 *            The alliance to configure the auto builder for.
	 */
	public void configureAutoBuilder(DriverStation.Alliance alliance) {
		if (alliance == null) {
			System.out.println("BAD! Allicance builder run without selected alliance");
			return;
		}
		Auto.setupPathPlanner(drivetrain, alliance);
		System.out.println("Configured path planner");

		NamedCommands.registerCommand("LockWheelsState", new LockWheelsCommand(drivetrain));
		auto = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto", auto);
		robotContainer.setAutoChooser(auto);
	}

	private Command resetPose() {
		return new InstantCommand(() -> {
			drivetrain.resetOdometry(pose.getSelected());
			drivetrain.resetLastAngleScalar();
		}).ignoringDisable(true);
	}
}
