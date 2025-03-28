package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
	public static Pose2d flipPoseAlliance(Pose2d pose) {
		return new Pose2d(flipTranslationAlliance(pose.getTranslation()), pose.getRotation().rotateBy(Rotation2d.k180deg));
	}

	/**
	 * Flips a translation to the other alliance.
	 *
	 * @param translation
	 *            Translation to flip.
	 * @return
	 *         Flipped translation.
	 */
	public static Translation2d flipTranslationAlliance(Translation2d translation) {
		return new Translation2d(FieldConstants.FIELD_LAYOUT.getFieldLength() - translation.getX(), FieldConstants.FIELD_LAYOUT.getFieldWidth() - translation.getY());
	}

	/**
	 * Flips a transform across the X axis (inverts the Y axis).
	 *
	 * @param transform
	 *            Transform to flip.
	 * @return
	 *         Flipped transform.
	 */
	public static Transform2d flipTransformY(Transform2d transform) {
		return new Transform2d(transform.getMeasureX(), transform.getMeasureY().times(-1), transform.getRotation());
	}
}
