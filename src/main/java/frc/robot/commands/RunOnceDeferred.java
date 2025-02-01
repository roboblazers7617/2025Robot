package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Runs a runnable on the next run of the CommandScheduler.
 */
public class RunOnceDeferred extends Command {
	private final Runnable runnable;

	/**
	 * Creates a new RunOnceDeferred.
	 * 
	 * @param runnable
	 *            Runnable to run.
	 */
	public RunOnceDeferred(Runnable runnable) {
		this.runnable = runnable;
	}

	@Override
	public void execute() {
		runnable.run();
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
