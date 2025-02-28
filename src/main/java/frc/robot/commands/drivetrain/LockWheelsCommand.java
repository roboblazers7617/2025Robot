// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * A command that locks the wheels of the swerve drive to force it to remain stationary.
 */
public class LockWheelsCommand extends Command {
	/**
	 * Drivetrain to lock the wheels of.
	 */
	private final Drivetrain drivetrain;

	/**
	 * Creates a new LockWheelsState.
	 *
	 * @param drivetrain
	 *            Drivetrain to lock the wheels of.
	 */
	public LockWheelsCommand(Drivetrain drivetrain) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drivetrain);
		this.drivetrain = drivetrain;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		drivetrain.lock();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		// TODO: #111 (Max) Have you tested this that it is the behavior that you want? Are you able to exit this mode?
		return false;
	}
}
