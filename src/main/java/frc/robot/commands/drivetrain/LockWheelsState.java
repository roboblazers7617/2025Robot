// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/**
 * A state that locks the wheels of the swerve drive to force it to remain stationary
 */
public class LockWheelsState extends Command {
	/**
	 * Creates a new LockWheelsState.
	 */
	private final Drivetrain swerveDrive;

	public LockWheelsState(Drivetrain swerveDrive) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(swerveDrive);
		this.swerveDrive = swerveDrive;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		swerveDrive.lock();
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
		return false;
	}
}
