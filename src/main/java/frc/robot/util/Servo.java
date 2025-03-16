package frc.robot.util;

import edu.wpi.first.math.MathUtil;

/**
 * An extension of WPILib's {@link edu.wpi.first.wpilibj.Servo} that adds some nice features.
 */
public class Servo extends edu.wpi.first.wpilibj.Servo {
	/**
	 * Conversion factor for the motor position. Defaults to 1.
	 */
	private double positionConversionFactor;

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
		this.positionConversionFactor = 1.0;
	}

	/**
	 * Sets the position of the motor. This gets divided by the {@link #positionConversionFactor}
	 * and clamped to a range of [0-1].
	 *
	 * @param position
	 *            The position to set [0-{@link #positionConversionFactor}].
	 * @see edu.wpi.first.wpilibj.Servo#set(double)
	 */
	@Override
	public void setAngle(double position) {
		double convertedPosition = position / positionConversionFactor;
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
	 * @see edu.wpi.first.wpilibj.Servo#get()
	 */
	@Override
	public double getAngle() {
		return getPosition() * positionConversionFactor;
	}

	/**
	 * Sets the {@link #positionConversionFactor} for this servo.
	 *
	 * @param positionConversionFactor
	 *            The {@link #positionConversionFactor} for this servo.
	 * @return
	 *         This object for method chaining.
	 */
	public Servo setPositionConversionFactor(double positionConversionFactor) {
		this.positionConversionFactor = positionConversionFactor;
		return this;
	}
}
