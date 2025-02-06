package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import swervelib.SwerveDrive;

/**
 * Class that contains logic related to vision.
 */
public class Vision {
	private static AngularVelocity3d zeroAngularVelocity3d = new AngularVelocity3d(DegreesPerSecond.zero(), DegreesPerSecond.zero(), DegreesPerSecond.zero());
	private static Limelight limelight = new Limelight(VisionConstants.LIMELIGHT_NAME);

	/**
	 * Update the pose estimation inside of {@link SwerveDrive} with data from Limelight.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to write measurements to.
	 */
	public static void updatePoseEstimation(SwerveDrive swerveDrive) {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		// ANGULAR VELOCITY IS NEGLIGABLE SO PASS IN ZEROS
		limelight.getSettings().withRobotOrientation(new Orientation3d(swerveDrive.getGyroRotation3d(), zeroAngularVelocity3d)).save();

		// Get MegaTag2 pose
		// If the pose is present, a tag is detected, and the robot is not spinning over 720 degrees per second, add it to the pose estimator
		limelight.getPoseEstimator(true).getPoseEstimate().ifPresent((PoseEstimate poseEstimate) -> {
			if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && poseEstimate.tagCount > 0) {
				swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
			}
		});
	}
}
