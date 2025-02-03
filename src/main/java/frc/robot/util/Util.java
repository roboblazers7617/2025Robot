package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;

/**
 * General utilities.
 */
public class Util {
	/**
	 * Checks if the alliance is red, defaults to false if alliance isn't available.
	 *
	 * @return
	 *         true if the red alliance, false if blue. Defaults to false if none is available.
	 */
	public static boolean isRedAlliance() {
		var alliance = DriverStation.getAlliance();
		return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
	}
}
