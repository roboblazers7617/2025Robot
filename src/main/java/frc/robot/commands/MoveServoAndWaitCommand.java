package frc.robot.commands;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Servo;

/**
 * Command to set a {@link Servo}'s position and wait for it to reach it. The Servo's position
 * conversion factor must convert to degrees for this to work.
 */
public class MoveServoAndWaitCommand extends Command {
	/**
	 * The servo to control.
	 */
	private final Servo servo;
	/**
	 * The position the servo started at.
	 */
	private double startPosition;
	/**
	 * The position to set the servo to.
	 */
	private final double endPosition;
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
	public MoveServoAndWaitCommand(Servo servo, double position) {
		this.servo = servo;
		this.endPosition = position;
		this.timer = new Timer();
	}

	@Override
	public void initialize() {
		timer.restart();
		startPosition = servo.getAngle();
		servo.setAngle(endPosition);
	}

	@Override
	public void end(boolean interrupted) {
		timer.stop();
	}

	@Override
	public boolean isFinished() {
		double moveTime = Math.abs(endPosition - startPosition) * (1 / servo.getRotationSpeed().in(DegreesPerSecond));
		return timer.hasElapsed(moveTime);
	}
}
