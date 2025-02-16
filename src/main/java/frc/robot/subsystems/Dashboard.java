// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.RunOnceDeferred;
import frc.robot.commands.drivetrain.LockWheelsCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * A class that sets up the driverstation dashboard for the robot.
 */
// TODO: #104 (Brandon) Why is the Dashboard a Subsystem?
public class Dashboard extends SubsystemBase {
	final Drivetrain drivetrain;
	final RobotContainer robotContainer;
	final SendableChooser<DriverStation.Alliance> alliancePicker;
	final SendableChooser<Pose2d> pose;
	SendableChooser<Command> auto;
	/**
	 * Field2d used to preview and monitor auto paths.
	 */
	private final Field2d autoFieldPreview;
	/**
	 * Field2d used to monitor the robot state in teleop.
	 */
	private final Field2d teleopFieldPreview;

	/** Creates a new Dashboard. */
	public Dashboard(Drivetrain drivetrain, RobotContainer robotContainer) {
		this.drivetrain = drivetrain;
		this.robotContainer = robotContainer;

		autoFieldPreview = new Field2d();
		teleopFieldPreview = new Field2d();

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

		pose.onChange((pose) -> {
			autoFieldPreview.getObject("Selected Pose")
					.setPose(pose);
		});

		PathPlannerLogging.setLogActivePathCallback((List<Pose2d> activePath) -> {
			autoFieldPreview.getObject("PathPlanner Path")
					.setPoses(activePath);
			teleopFieldPreview.getObject("PathPlanner Path")
					.setPoses(activePath);
		});
		PathPlannerLogging.setLogCurrentPoseCallback((Pose2d currentPose) -> {
			autoFieldPreview.getObject("PathPlanner Current Pose")
					.setPose(currentPose);
		});
		PathPlannerLogging.setLogTargetPoseCallback((Pose2d targetPose) -> {
			autoFieldPreview.getObject("PathPlanner Target")
					.setPose(targetPose);
			teleopFieldPreview.getObject("PathPlanner Target")
					.setPose(targetPose);
		});

		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("selected").setString("None");
		NetworkTableInstance.getDefault().getTable("SmartDashboard/Alliance").getEntry("active").setString("None");
		SmartDashboard.putData("Alliance", alliancePicker);
		SmartDashboard.putData("Pose", pose);
		SmartDashboard.putData("Reset pose to selected position", resetPose());
		SmartDashboard.putData("Auto Field Preview", autoFieldPreview);
		SmartDashboard.putData("Teleop Field Preview", teleopFieldPreview);
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

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		// System.out.println(alliancePicker.getSelected());
		autoFieldPreview.setRobotPose(drivetrain.getSwerveDrive().field.getRobotPose());
		teleopFieldPreview.setRobotPose(drivetrain.getSwerveDrive().field.getRobotPose());
		autoFieldPreview.getObject("XModules")
				.setPoses(drivetrain.getSwerveDrive().field.getObject("XModules").getPoses());
		teleopFieldPreview.getObject("XModules")
				.setPoses(drivetrain.getSwerveDrive().field.getObject("XModules").getPoses());
	}
}
