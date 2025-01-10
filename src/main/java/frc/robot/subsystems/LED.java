package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

/**
 * Subsystem to control the robot's addressable LEDs.
 */
public class LED extends SubsystemBase {
	/**
	 * Object to control the LED hardware.
	 */
	private final AddressableLED led = new AddressableLED(LEDConstants.LED_PORT);
	/**
	 * LED buffer to hold the LED values.
	 */
	private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT);

	/**
	 * LED pattern to display.
	 */
	private LEDPattern currentPattern;

	/**
	 * Creates a new LED subsystem.
	 */
	public LED() {
		// Set the length of the LED strip
		led.setLength(ledBuffer.getLength());

		// Set the LED data
		led.setData(ledBuffer);
		led.start();

		currentPattern = LEDPattern.rainbow(255, 128).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LEDConstants.LED_SPACING);
	}

	@Override
	public void periodic() {
		// Update the current LED pattern
		currentPattern.applyTo(ledBuffer);
		led.setData(ledBuffer);
	}
}
