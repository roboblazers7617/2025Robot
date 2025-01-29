// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dashboard extends SubsystemBase {
	Drivetrain drivetrain;
	SendableChooser<DriverStation.Alliance> alliancePicker;
	SendableChooser<Pose2d> pose;
	SendableChooser<Command> auto;

	/** Creates a new Dashboard. */
	public Dashboard(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		alliancePicker = new SendableChooser<DriverStation.Alliance>();
		alliancePicker.setDefaultOption("Red", DriverStation.Alliance.Red);
		alliancePicker.addOption("Blue", DriverStation.Alliance.Blue);

		pose = new SendableChooser<Pose2d>();
		pose.setDefaultOption("position 1", new Pose2d(0, 0, new Rotation2d(0)));
		pose.addOption("position 2", new Pose2d(0, 0, new Rotation2d(0)));

		auto = new SendableChooser<Command>();
		auto.setDefaultOption("Do Nothing", new InstantCommand());
		auto.addOption("Drive Forward", new InstantCommand());

		SmartDashboard.putData("Alliance", alliancePicker);
		SmartDashboard.putData("Pose", pose);
		// SmartDashboard.putData("Set Pose", setPose());
		SmartDashboard.putData("Auto", auto);
		// SmartDashboard.putData("Set Auto", setAuto());
	}
}
