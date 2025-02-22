package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import swervelib.SwerveDrive;
import io.github.roboblazers7617.limelight.Limelight;
import io.github.roboblazers7617.limelight.LimelightSettings;
import io.github.roboblazers7617.limelight.PoseEstimator;
import io.github.roboblazers7617.limelight.LimelightSettings.ImuMode;
import io.github.roboblazers7617.limelight.PoseEstimate;

/**
 * Class that handles vision logic.
 */
public class Vision {
	/**
	 * {@link SwerveDrive} to write vision updates to.
	 */
	private final SwerveDrive swerveDrive;
	/**
	 * {@link Limelight} 4 mounted on the front of the robot.
	 */
	private final Limelight frontLimelight;
	/**
	 * {@link Limelight} 3G mounted on the front of the robot.
	 */
	private final Limelight backLimelight;
	/**
	 * {@link PoseEstimator} for the {@link Limelight} on the front of the robot.
	 */
	private final PoseEstimator frontPoseEstimator;
	/**
	 * {@link PoseEstimator} for the {@link Limelight} on the front of the robot.
	 */
	private final PoseEstimator backPoseEstimator;

	/**
	 * Creates a new Vision.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to update.
	 */
	public Vision(SwerveDrive swerveDrive) {
		this.swerveDrive = swerveDrive;
		frontLimelight = new Limelight(VisionConstants.FRONT_LIMELIGHT_NAME);
		backLimelight = new Limelight(VisionConstants.BACK_LIMELIGHT_NAME);
		frontPoseEstimator = frontLimelight.makePoseEstimator(VisionConstants.POSE_ESTIMATOR_TYPE);
		backPoseEstimator = backLimelight.makePoseEstimator(VisionConstants.POSE_ESTIMATOR_TYPE);

		frontLimelight.settings.withImuMode(VisionConstants.DISABLED_IMU_MODE).withProcessedFrameFrequency(VisionConstants.DISABLED_UPDATE_FREQUENCY).save();
	}

	public Command onEnableCommand() {
		return Commands.runOnce(() -> {
			frontLimelight.settings.withImuMode(VisionConstants.ENABLED_IMU_MODE).withProcessedFrameFrequency(0).save();
		});
	}

	public Command onDisableCommand() {
		return Commands.runOnce(() -> {
			frontLimelight.settings.withImuMode(VisionConstants.DISABLED_IMU_MODE).withProcessedFrameFrequency(VisionConstants.DISABLED_UPDATE_FREQUENCY).save();
		});
	}

	// TODO: #118 (Lukas) Please incorporate setup of LL4 parameters
	/**
	 * Update the pose estimation inside of {@link #swerveDrive} with data from Limelight.
	 */
	public void updatePoseEstimation() {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		frontLimelight.setRobotOrientation(swerveDrive.getGyroRotation3d());
		backLimelight.setRobotOrientation(swerveDrive.getGyroRotation3d());

		// Get pose estimates from Limelights
		PoseEstimate[] frontLimelightPoseEstimates = frontPoseEstimator.getBotPoseEstimates();
		PoseEstimate[] backLimelightPoseEstimates = backPoseEstimator.getBotPoseEstimates();

		for (PoseEstimate poseEstimate : frontLimelightPoseEstimates) {
			// Don't try to use a null PoseEstimate
			if (poseEstimate != null) {
				// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
				if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && poseEstimate.tagCount > 0) {
					swerveDrive.addVisionMeasurement(poseEstimate.getPose2d(), poseEstimate.getTimestampSeconds(), VecBuilder.fill(.7, .7, 9999999));
				}
			}
		}
		for (PoseEstimate poseEstimate : backLimelightPoseEstimates) {
			// Don't try to use a null PoseEstimate
			if (poseEstimate != null) {
				// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
				if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && poseEstimate.tagCount > 0) {
					swerveDrive.addVisionMeasurement(poseEstimate.getPose2d(), poseEstimate.getTimestampSeconds(), VecBuilder.fill(.7, .7, 9999999));
				}
			}
		}
	}
}
