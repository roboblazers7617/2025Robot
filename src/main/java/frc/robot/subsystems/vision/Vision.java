package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;

/**
 * Class that contains logic related to vision.
 */
public class Vision {
	/**
	 * Update the pose estimation inside of {@link SwerveDrive} with data from Limelight.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to write measurements to.
	 */
	public static void updatePoseEstimation(SwerveDrive swerveDrive) {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAME, swerveDrive.getYaw().getDegrees(), 0, swerveDrive.getPitch().getDegrees(), 0, swerveDrive.getRoll().getDegrees(), 0);

		// Get a pose estimate from Limelight
		LimelightHelpers.PoseEstimate limelightPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);

		// Don't try to use a null PoseEstimate
		if (limelightPoseEstimate == null) {
			return;
		}

		// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
		if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && limelightPoseEstimate.tagCount > 0) {
			swerveDrive.addVisionMeasurement(limelightPoseEstimate.pose, limelightPoseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
		}
	}
}
