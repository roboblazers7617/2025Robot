// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffector.EndEffector;
import frc.robot.subsystems.drivetrain.Drivetrain;

/** This stores Automations, primarly to be used with Autonomous routines. */
public class Automations {
	public void createNamedCommands(Drivetrain drivetrain, EndEffector endEffector, Elevator elevator) {
		NamedCommands.registerCommand("Intake Coral", elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION).andThen(endEffector.CoralIntake()));
		NamedCommands.registerCommand("Intake Algae L2", elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_2).andThen(endEffector.AlgaeIntake()));
		NamedCommands.registerCommand("Intake Algae L3", elevator.SetPositionCommand(ArmPosition.INTAKE_ALGAE_LEVEL_3).andThen(endEffector.AlgaeIntake()));

		NamedCommands.registerCommand("Elevator L1 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_1));
		NamedCommands.registerCommand("Elevator L2 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_2));
		NamedCommands.registerCommand("Elevator L3 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_3));
		NamedCommands.registerCommand("Elevator L4 Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4));
		NamedCommands.registerCommand("Elevator Processor Position", elevator.SetPositionCommand(ArmPosition.OUTTAKE_ALGAE_PROCESSOR));

		NamedCommands.registerCommand("Stow Empty", elevator.SetPositionCommand(ArmPosition.INTAKE_CORAL_CORAL_STATION));

		NamedCommands.registerCommand("Score Coral L1-3", endEffector.CoralOuttake());
		NamedCommands.registerCommand("Score Coral L4", endEffector.CoralOuttake().alongWith(elevator.SetPositionCommand(ArmPosition.OUTTAKE_CORAL_LEVEL_4_HIGH)));
		NamedCommands.registerCommand("Score Algae", endEffector.AlgaeOuttake());
	}
}
