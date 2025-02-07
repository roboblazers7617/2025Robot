package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.FieldConstants;

/**
 * Utility classes for working with poses.
 */
public class PoseUtil {
	/**
	 * Flips a pose to the other alliance.
	 *
	 * @param pose
	 *            Pose to flip.
	 * @return
	 *         Flipped pose.
	 */
	public static Pose2d flipPose(Pose2d pose) {
		// TODO: (Max) If you are flipping between alliances, don't you also need to flip the rotation?
		// Think about if you are facing the reef on the blue side, if you flip to be facing the reef on the
		// red side you need to change the rotation
		return new Pose2d(flipTranslation(pose.getTranslation()), pose.getRotation());
	}

	/**
	 * Flips a translation to the other alliance.
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation2d flipTranslation(Translation2d translation) {
		// TODO: (Max) I think you need to also adjust the Y coordinate. Look at a field map
		return new Translation2d(FieldConstants.FIELD_LAYOUT.getFieldLength() - translation.getX(), translation.getY());
	}
}
