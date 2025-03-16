package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MoveServoAndWaitCommand;

/**
 * An extension of WPILib's {@link edu.wpi.first.wpilibj.Servo} that adds some nice features.
 */
public class Servo extends edu.wpi.first.wpilibj.Servo {
	/**
	 * Conversion factor for the motor position. Defaults to 180 degrees.
	 */
	private Angle positionConversionFactor;
	/**
	 * How fast does the servo usually rotate? Used to calculate when the servo should have reached
	 * its setpoint. Defaults to 315 degrees per second.
	 */
	private AngularVelocity rotationSpeed;

	/**
	 * Creates a new Servo.
	 *
	 * @param channel
	 *            The PWM channel to which the servo is attached.
	 *            <p>
	 *            0-9 are on-board, 10-19 are on the MXP port.
	 */
	public Servo(final int channel) {
		super(channel);

		// Default values
		this.positionConversionFactor = Degrees.of(180.0);
		this.rotationSpeed = DegreesPerSecond.of(315);
	}

	/**
	 * Sets the position of the motor. This gets divided by the {@link #positionConversionFactor}
	 * and clamped to a range of [0-1].
	 *
	 * @param position
	 *            The position to set [0-{@link #positionConversionFactor}].
	 * @see edu.wpi.first.wpilibj.Servo#set(double)
	 */
	public void setAngleMeasure(Angle position) {
		double convertedPosition = position.div(positionConversionFactor).magnitude();
		setPosition(MathUtil.clamp(convertedPosition, 0.0, 1.0));
	}

	/**
	 * Gets the position of the servo. This will be in the range specified by the {@link #positionConversionFactor}.
	 *
	 * @apiNote
	 *          Since the RIO just blindly sends PWM data to the servo, it doesn't recieve any
	 *          feedback about where it is, so this just returns the commanded position, and, in
	 *          the case that the servo can't reach its target, this will be incorrect.
	 * @return
	 *         The position of the servo [0-{@link #positionConversionFactor}].
	 * @see edu.wpi.first.wpilibj.Servo#getPosition()
	 */
	public Angle getAngleMeasure() {
		return positionConversionFactor.times(getPosition());
	}

	/**
	 * Moves this servo and finishes after the servo has reached its target position. Estimates the
	 * amount of time this will take using the {@link #rotationSpeed}.
	 *
	 * @param position
	 *            The position to set the servo to.
	 * @return
	 *         Command to run.
	 */
	public Command moveCommand(Angle position) {
		return new MoveServoAndWaitCommand(this, position);
	}

	/**
	 * Sets the {@link #positionConversionFactor} for this Servo. This states how much the servo can
	 * turn over it's entire travel (eg. 180 degrees for a half-turn servo).
	 *
	 * @param positionConversionFactor
	 *            The {@link #positionConversionFactor} to set.
	 * @return
	 *         This object for method chaining.
	 */
	public Servo setPositionConversionFactor(Angle positionConversionFactor) {
		this.positionConversionFactor = positionConversionFactor;
		return this;
	}

	/**
	 * Sets the {@link #rotationSpeed} of this Servo.
	 *
	 * @param rotationSpeed
	 *            The {@link #rotationSpeed} to set.
	 * @return
	 *         This object for method chaining.
	 */
	public Servo setRotationSpeed(AngularVelocity rotationSpeed) {
		this.rotationSpeed = rotationSpeed;
		return this;
	}

	/**
	 * Gets the {@link #rotationSpeed} of this Servo.
	 *
	 * @return
	 *         The {@link #rotationSpeed} of this Servo.
	 */
	public AngularVelocity getRotationSpeed() {
		return rotationSpeed;
	}
}
