package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
	private final AddressableLED led;
	private final AddressableLEDBuffer ledBuffer;
	private final AddressableLEDBufferView leftLED;
	private final AddressableLEDBufferView rightLED;

	private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
	private final LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LEDConstants.LED_SPACING);
	private final LEDPattern scrollingRainbow2 = LEDPattern.rainbow(255, 255).reversed().scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LEDConstants.LED_SPACING);
	private final LEDPattern red = LEDPattern.solid(Color.kRed);

	public LED() {
		led = new AddressableLED(LEDConstants.LED_PWM_HEADER);
		ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_AMMOUNT_TOTAL);
		leftLED = ledBuffer.createView(LEDConstants.LEFT_BUFFER_INDEX_START, LEDConstants.LEFT_BUFFER_INDEX_END);
		rightLED = ledBuffer.createView(LEDConstants.RIGHT_BUFFER_INDEX_START, LEDConstants.RIGHT_BUFFER_INDEX_END).reversed();
		led.setLength(ledBuffer.getLength());
		led.setData(ledBuffer);
		led.start();
	}

	@Override
	public void periodic() {
		led.setData(ledBuffer);
	}

	public Command runPattern(LEDPattern leftPattern, LEDPattern rightPattern) {
		return run(() -> {
			leftPattern.applyTo(leftLED);
			rightPattern.applyTo(rightLED);
		});
	}

	public Command red() {
		return runPattern(red, rainbow);
	}

	public Command rainbow() {
		return runPattern(rainbow, rainbow);
	}

	public Command scrollingRainbow() {
		return runPattern(scrollingRainbow, scrollingRainbow2);
	}
}