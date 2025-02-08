package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;
import limelight.Limelight;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;
import limelight.networktables.LimelightSettings.ImuMode;
import swervelib.SwerveDrive;

/**
 * Class that handles vision logic.
 */
public class Vision {
	/**
	 * An AngularVelocity3d with all values set to zero.
	 */
	private static final AngularVelocity3d zeroAngularVelocity3d = new AngularVelocity3d(DegreesPerSecond.zero(), DegreesPerSecond.zero(), DegreesPerSecond.zero());
	/**
	 * The front Limelight.
	 */
	private final Limelight frontLimelight;
	/**
	 * The back Limelight.
	 */
	private final Limelight backLimelight;
	/**
	 * Pose estimator for the {@link frontLimeLight}.
	 */
	private final LimelightPoseEstimator frontPoseEstimator;
	/**
	 * Pose estimator for the {@link backLimelight}.
	 */
	private final LimelightPoseEstimator backPoseEstimator;

	/**
	 * Creates a new Vision.
	 */
	public Vision() {
		// Create the limelights
		frontLimelight = new Limelight(VisionConstants.FRONT_LIMELIGHT_NAME);
		backLimelight = new Limelight(VisionConstants.BACK_LIMELIGHT_NAME);

		// Create the pose estimators.
		frontPoseEstimator = frontLimelight.getPoseEstimator(true);
		backPoseEstimator = backLimelight.getPoseEstimator(true);

		// Tell the LL4 to only use provided gyro values
		backLimelight.getSettings()
				.withImuMode(ImuMode.ExternalImu);
	}

	/**
	 * Update the pose estimation inside of {@link SwerveDrive} with data from Limelight.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to use.
	 */
	public void updatePoseEstimation(SwerveDrive swerveDrive) {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		// ANGULAR VELOCITY IS NEGLIGABLE SO PASS IN ZEROS
		frontLimelight.getSettings()
				.withRobotOrientation(new Orientation3d(swerveDrive.getGyroRotation3d(), zeroAngularVelocity3d))
				.save();
		backLimelight.getSettings()
				.withRobotOrientation(new Orientation3d(swerveDrive.getGyroRotation3d(), zeroAngularVelocity3d))
				.save();

		// Get MegaTag2 pose
		// If the pose is present, a tag is detected, and the robot is not spinning over 720 degrees per second, add it to the pose estimator
		Optional<PoseEstimate> frontPoseEstimate = frontPoseEstimator.getPoseEstimate();
		frontPoseEstimate.ifPresent((PoseEstimate poseEstimate) -> {
			if (Math.abs(RadiansPerSecond.of(swerveDrive.getRobotVelocity().omegaRadiansPerSecond).in(DegreesPerSecond)) < 720 && poseEstimate.tagCount > 0) {
				swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
			}
		});

		Optional<PoseEstimate> backPoseEstimate = backPoseEstimator.getPoseEstimate();
		backPoseEstimate.ifPresent((PoseEstimate poseEstimate) -> {
			if (Math.abs(RadiansPerSecond.of(swerveDrive.getRobotVelocity().omegaRadiansPerSecond).in(DegreesPerSecond)) < 720 && poseEstimate.tagCount > 0) {
				swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds, VecBuilder.fill(.7, .7, 9999999));
			}
		});
	}
}
