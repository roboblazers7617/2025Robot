package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;

import frc.robot.Constants.VisionConstants;

import swervelib.SwerveDrive;
import io.github.roboblazers7617.limelight.Limelight;
import io.github.roboblazers7617.limelight.PoseEstimator;
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
	// private final Limelight backLimelight;
	/**
	 * {@link PoseEstimator} for the {@link Limelight} on the front of the robot.
	 */
	private final PoseEstimator frontPoseEstimator;
	/**
	 * {@link PoseEstimator} for the {@link Limelight} on the front of the robot.
	 */
	// private final PoseEstimator backPoseEstimator;
	/**
	 * Stores the previous gyro heading
	 */
	private Rotation2d previousHeading;

	/**
	 * Creates a new Vision.
	 *
	 * @param swerveDrive
	 *            {@link SwerveDrive} to update.
	 */
	public Vision(SwerveDrive swerveDrive) {
		this.swerveDrive = swerveDrive;
		frontLimelight = new Limelight(VisionConstants.FRONT_LIMELIGHT_NAME);
		// backLimelight = new Limelight(VisionConstants.BACK_LIMELIGHT_NAME);
		frontPoseEstimator = frontLimelight.makePoseEstimator(VisionConstants.POSE_ESTIMATOR_TYPE);
		// backPoseEstimator = backLimelight.makePoseEstimator(VisionConstants.POSE_ESTIMATOR_TYPE);

		/* backLimelight.settings.withImuMode(VisionConstants.DISABLED_IMU_MODE).withProcessedFrameFrequency(VisionConstants.DISABLED_UPDATE_FREQUENCY).save(); */

		previousHeading = swerveDrive.getOdometryHeading();

		// Run enable command on enable
		// RobotModeTriggers.teleop()
		// .onTrue(onEnableCommand());
	}

	/*
	 * public Command onEnableCommand() {
	 * return Commands.runOnce(() -> {
	 * backLimelight.settings.withImuMode(VisionConstants.ENABLED_IMU_MODE).withProcessedFrameFrequency(0).save();
	 * }).ignoringDisable(true);
	 * }
	 * public Command onDisableCommand() {
	 * return Commands.runOnce(() -> {
	 * backLimelight.settings.withImuMode(VisionConstants.DISABLED_IMU_MODE).withProcessedFrameFrequency(VisionConstants.DISABLED_UPDATE_FREQUENCY).save();
	 * }).ignoringDisable(true);
	 * }
	 */

	/**
	 * Command to call when the robot is enabled.
	 * <p>
	 * Updates the AprilTag ID filter.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command onEnableCommand() {
		return Commands.runOnce(() -> {
			// Update the AprilTag ID filter
			Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
			if (alliance.isPresent()) {
				setTagFilterAlliance(alliance.get());
			}
		});
	}

	/**
	 * Sets the AprilTag ID filter to the appropriate set for the given alliance.
	 *
	 * @param alliance
	 *            The alliance to set it for.
	 */
	public void setTagFilterAlliance(DriverStation.Alliance alliance) {
		switch (alliance) {
			case Blue:
				frontLimelight.settings.withArilTagIdFilter(VisionConstants.BLUE_TAG_ID_FILTER)
						.save();
				break;

			case Red:
				frontLimelight.settings.withArilTagIdFilter(VisionConstants.RED_TAG_ID_FILTER)
						.save();
				break;
		}
	}

	/**
	 * Update the pose estimation inside of {@link #swerveDrive} with data from Limelight.
	 */
	public void updatePoseEstimation() {
		// Get robot pose from YAGSL and use it to set the orientation in Limelight
		frontLimelight.setRobotOrientation(new Rotation3d(swerveDrive.getOdometryHeading()));
		// backLimelight.setRobotOrientation(swerveDrive.getGyroRotation3d());

		// Get pose estimates from Limelights
		PoseEstimate[] frontLimelightPoseEstimates = frontPoseEstimator.getBotPoseEstimates();
		// PoseEstimate[] backLimelightPoseEstimates = backPoseEstimator.getBotPoseEstimates();

		for (PoseEstimate poseEstimate : frontLimelightPoseEstimates) {
			// Don't try to use a null PoseEstimate
			if (poseEstimate != null && DriverStation.isEnabled() && Math.abs(swerveDrive.getOdometryHeading().minus(previousHeading).getDegrees()) < 50.0) {
				// Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
				if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && poseEstimate.tagCount > 0) {
					swerveDrive.addVisionMeasurement(poseEstimate.getPose2d(), poseEstimate.getTimestampSeconds(), VecBuilder.fill(.7, .7, 9999999));
				}
			}
		}

		/*
		 * for (PoseEstimate poseEstimate : backLimelightPoseEstimates) {
		 * // Don't try to use a null PoseEstimate
		 * if (poseEstimate != null) {
		 * // Only update vision if our angular velocity is less than 720 degrees per second and a tag was detected
		 * if (Math.abs(swerveDrive.getMaximumChassisAngularVelocity()) < 720 && poseEstimate.tagCount > 0) {
		 * swerveDrive.addVisionMeasurement(poseEstimate.getPose2d(), poseEstimate.getTimestampSeconds(), VecBuilder.fill(.7, .7, 9999999));
		 * }
		 * }
		 * }
		 */
		previousHeading = swerveDrive.getOdometryHeading();
	}
}
