package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A Pose2d that contains versions for the Blue and Red alliances.
 */
public class BiAlliancePose2d {
	/**
	 * The pose on the blue alliance.
	 */
	public final Pose2d bluePose;
	/**
	 * The pose on the red alliance.
	 */
	public final Pose2d redPose;

	/**
	 * Creates a BiAlliancePose2d from two poses.
	 *
	 * @param bluePose
	 *            The pose on the blue alliance.
	 * @param redPose
	 *            The pose on the red alliance.
	 */
	private BiAlliancePose2d(Pose2d bluePose, Pose2d redPose) {
		this.bluePose = bluePose;
		this.redPose = redPose;
	}

	/**
	 * Creates a BiAlliancePose2d from a pose on the blue alliance.
	 *
	 * @param bluePose
	 *            The pose on the blue alliance.
	 */
	public static BiAlliancePose2d fromBluePose(Pose2d bluePose) {
		return new BiAlliancePose2d(bluePose, PoseUtil.flipPose(bluePose));
	}

	/**
	 * Creates a BiAlliancePose2d from a pose on the red alliance.
	 *
	 * @param redPose
	 *            The pose on the red alliance.
	 */
	public static BiAlliancePose2d fromRedPose(Pose2d redPose) {
		return new BiAlliancePose2d(PoseUtil.flipPose(redPose), redPose);
	}

	/**
	 * Gets this pose on the blue alliance.
	 *
	 * @return
	 *         The pose on the blue alliance.
	 * @see #bluePose
	 */
	public Pose2d getBluePose() {
		return bluePose;
	}

	/**
	 * Gets this pose on the red alliance.
	 *
	 * @return
	 *         The pose on the red alliance.
	 * @see #redPose
	 */
	public Pose2d getRedPose() {
		return redPose;
	}

	/**
	 * Gets this pose for the current alliance.
	 *
	 * @return
	 *         The pose for the current alliance.
	 */
	public Pose2d getPoseByAlliance() {
		if (Util.isRedAlliance()) {
			return getRedPose();
		} else {
			return getBluePose();
		}
	}
}
