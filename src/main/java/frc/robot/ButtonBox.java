package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.roboblazers7617.buttonbox.ButtonBoxServer;
import io.github.roboblazers7617.buttonbox.controls.LEDMulticolor;

public class ButtonBox extends SubsystemBase {
	/**
	 * ButtonBoxServer to use to interface with the ButtonBox.
	 */
	private final ButtonBoxServer buttonBoxServer = new ButtonBoxServer();
	/**
	 * LED used to show coral/algae state.
	 */
	private final LEDMulticolor modeLED;
	/**
	 * The current hue of the {@link #modeLED}'s color cycling.
	 */
	private double hue = 0;

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
		modeLED = new LEDMulticolor("Mode LED");
		buttonBoxServer.addControl(modeLED);
		modeLED.setBrightness(1);
	}

	@Override
	public void periodic() {
		if (DriverStation.isDisabled()) {
			// Rainbow color cycle
			modeLED.setColor(Color.fromHSV((int) hue, 255, 255));
			hue += 0.25;
		} else if (DriverStation.isAutonomousEnabled()) {
			modeLED.setColor(Color.kRed);
		} else if (DriverStation.isTeleopEnabled()) {
			switch (robotContainer.getGamepieceMode()) {
				case CORAL_MODE:
					modeLED.setColor(Color.kWhite);
					break;

				case ALGAE_MODE:
					modeLED.setColor(new Color(0, 1, 0.64));
					break;
			}
		}
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
