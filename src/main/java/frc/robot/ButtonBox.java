package frc.robot;

import io.github.roboblazers7617.buttonbox.ButtonBoxServer;
import io.github.roboblazers7617.buttonbox.controls.Joystick;
import io.github.roboblazers7617.buttonbox.controls.Knob;

public class ButtonBox {
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
	 * Creates a new ButtonBox.
	 */
	public ButtonBox() {
		// Create controls
		driverJoystick = new Joystick("Driver Joystick");
		buttonBoxServer.addControl(driverJoystick);

		driverSteeringKnob = new Knob("Driver Steering Knob");
		buttonBoxServer.addControl(driverSteeringKnob);
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
}
