package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;

/**
 * Class that handles vision logic.
 */
public class Vision {
	/**
	 * SwerveDrive to write vision updates to.
	 */
	private final SwerveDrive swerveDrive;

	/**
	 * Creates a new Vision.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to update.
	 */
	public Vision(SwerveDrive swerveDrive) {
		this.swerveDrive = swerveDrive;
	}

	/**
	 * Update the pose estimation inside of {@link #swerveDrive} with data from Limelight.
	 */
	public void updatePoseEstimation() {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		LimelightHelpers.SetRobotOrientation(VisionConstants.FRONT_LIMELIGHT_NAME, swerveDrive.getYaw().getDegrees(), 0, swerveDrive.getPitch().getDegrees(), 0, swerveDrive.getRoll().getDegrees(), 0);
		LimelightHelpers.SetRobotOrientation(VisionConstants.BACK_LIMELIGHT_NAME, swerveDrive.getYaw().getDegrees(), 0, swerveDrive.getPitch().getDegrees(), 0, swerveDrive.getRoll().getDegrees(), 0);

		// Get pose estimates from Limelights
		LimelightHelpers.PoseEstimate frontLimelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.FRONT_LIMELIGHT_NAME);
		LimelightHelpers.PoseEstimate backLimelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.BACK_LIMELIGHT_NAME);

		// Don't try to use a null PoseEstimate
		if (frontLimelightPoseEstimate != null) {
			// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
			if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && frontLimelightPoseEstimate.tagCount > 0) {
				swerveDrive.addVisionMeasurement(frontLimelightPoseEstimate.pose, frontLimelightPoseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
			}
		}
		if (backLimelightPoseEstimate != null) {
			// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
			if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && backLimelightPoseEstimate.tagCount > 0) {
				swerveDrive.addVisionMeasurement(backLimelightPoseEstimate.pose, backLimelightPoseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
			}
		}
	}
}
