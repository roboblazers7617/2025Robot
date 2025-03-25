package frc.robot.commands;

import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Servo;

/**
 * Command to set a {@link Servo}'s position and wait for it to reach it.
 */
public class MoveServoAndWaitCommand extends Command {
	/**
	 * The servo to control.
	 */
	private final Servo servo;
	/**
	 * The position the servo started at.
	 */
	private Angle startPosition;
	/**
	 * The position to set the servo to.
	 */
	private final Angle endPosition;
	/**
	 * The timer used to estimate how long the Servo will take to move.
	 */
	private final Timer timer;

	/**
	 * Creates a new MoveServoAndWaitCommand.
	 *
	 * @param servo
	 *            The Servo to control.
	 * @param position
	 *            The position to set the Servo to.
	 */
	public MoveServoAndWaitCommand(Servo servo, Angle position) {
		this.servo = servo;
		this.endPosition = position;
		this.timer = new Timer();
	}

	@Override
	public void initialize() {
		timer.restart();
		startPosition = servo.getAngleMeasure();
		servo.setAngleMeasure(endPosition);
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
	}

	@Override
	public boolean isFinished() {
		double distance = startPosition.minus(endPosition).abs(Revolutions);
		double rate = 1 / servo.getRotationSpeed().in(RevolutionsPerSecond);
		return timer.hasElapsed(distance * rate);
	}
}
