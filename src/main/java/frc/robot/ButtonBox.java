package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.roboblazers7617.buttonbox.ButtonBoxServer;
import io.github.roboblazers7617.buttonbox.controls.Joystick;
import io.github.roboblazers7617.buttonbox.controls.Knob;
import io.github.roboblazers7617.buttonbox.controls.LEDMulticolor;

public class ButtonBox extends SubsystemBase {
	/**
	 * ButtonBoxServer to use to interface with the ButtonBox.
	 */
	private final ButtonBoxServer buttonBoxServer = new ButtonBoxServer();
	/**
	 * Joystick used to drive the robot.
	 */
	private final Joystick driverJoystick;
	/**
	 * Knob used to turn the robot.
	 */
	private final Knob driverSteeringKnob;
	/**
	 * LED used to show coral/algae state.
	 */
	private final LEDMulticolor modeLED;

	/**
	 * RobotContainer used to poll subsystem states.
	 */
	private final RobotContainer robotContainer;

	/**
	 * Creates a new ButtonBox.
	 *
	 * @param robotContainer
	 *            The RobotContainer to get subsystem states from.
	 */
	public ButtonBox(RobotContainer robotContainer) {
		this.robotContainer = robotContainer;

		// Create controls
		driverJoystick = new Joystick("Driver Joystick");
		buttonBoxServer.addControl(driverJoystick);

		driverSteeringKnob = new Knob("Driver Steering Knob");
		buttonBoxServer.addControl(driverSteeringKnob);

		modeLED = new LEDMulticolor("Mode LED");
		buttonBoxServer.addControl(modeLED);
		modeLED.setBrightness(1);
	}

	@Override
	public void periodic() {
		switch (robotContainer.getGamepieceMode()) {
			case CORAL_MODE:
				modeLED.setColor(Color.kWhite);
				break;

			case ALGAE_MODE:
				modeLED.setColor(new Color(0, 1, 0.64));
				break;
		}
	}

	/**
	 * Gets the driver Joystick.
	 *
	 * @return
	 *         Driver Joystick.
	 */
	public Joystick getDriverJoystick() {
		return driverJoystick;
	}

	/**
	 * Gets the driver steering Knob.
	 *
	 * @return
	 *         Driver steering Knob.
	 */
	public Knob getDriverSteeringKnob() {
		return driverSteeringKnob;
	}

	/**
	 * Gets the mode LED.
	 *
	 * @return
	 *         Mode LED.
	 */
	public LEDMulticolor getModeLED() {
		return modeLED;
	}
}
