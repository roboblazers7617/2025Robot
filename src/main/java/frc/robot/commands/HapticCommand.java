package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

/**
 * Triggers controller rumble for a specified amount of time.
 */
public class HapticCommand extends Command {
	/**
	 * The timer used to time the rumble.
	 */
	private Timer timer = new Timer();

	/**
	 * Controller to rumble.
	 */
	private final GenericHID controller;
	/**
	 * Type of rumble to use.
	 */
	private final RumbleType type;
	/**
	 * Rumble strength [0-1].
	 */
	private final double strength;
	/**
	 * Duration to rumble for.
	 */
	private final Time duration;

	/**
	 * Creates a Command that triggers controller rumble for a specified amount of time.
	 *
	 * @param controller
	 *            Controller to rumble.
	 * @param type
	 *            Rumble type to set.
	 * @param strength
	 *            Rumble strength [0-1].
	 * @param duration
	 *            Rumble duration.
	 */
	public HapticCommand(GenericHID controller, RumbleType type, double strength, Time duration) {
		this.controller = controller;
		this.type = type;
		this.strength = strength;
		this.duration = duration;
	}

	/**
	 * Creates a Command that triggers controller rumble for a specified amount of time.
	 *
	 * @param controller
	 *            Controller to rumble.
	 * @param type
	 *            Rumble type to set.
	 * @param strength
	 *            Rumble strength [0-1].
	 * @param duration
	 *            Rumble duration.
	 */
	public HapticCommand(CommandGenericHID controller, RumbleType type, double strength, Time duration) {
		this(controller.getHID(), type, strength, duration);
	}

	@Override
	public void initialize() {
		timer.restart();
		controller.setRumble(type, strength);
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
		controller.setRumble(type, 0);
	}

	@Override
	public boolean isFinished() {
		return timer.hasElapsed(duration.in(Seconds));
	}

	@Override
	public boolean runsWhenDisabled() {
		return true;
	}
}
