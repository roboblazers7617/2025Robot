package frc.robot.subsystems.drivetrain;

import java.util.List;
import java.util.function.Supplier;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.drivetrain.Drivetrain.TranslationOrientation;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import swervelib.SwerveInputStream;

/**
 * Class that handles controlling the {@link Drivetrain} with HID controllers.
 */
public class DrivetrainControls {
	/**
	 * The Drivetrain to control.
	 */
	private final Drivetrain drivetrain;
	/**
	 * Speed multiplier used for scaling controller inputs (0, 1].
	 */
	private double speedMultiplier;

	/**
	 * Creates a new DrivetrainControls.
	 *
	 * @param drivetrain
	 *            The Drivetrain to control.
	 */
	public DrivetrainControls(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		// Set things to their default states
		resetSpeedMultiplier();
	}

	/**
	 * Sets the controller speed multiplier.
	 *
	 * @param speedMultiplier
	 *            Multiplier to set (0, 1].
	 */
	public void setSpeedMultiplier(double speedMultiplier) {
		this.speedMultiplier = speedMultiplier;
	}

	/**
	 * Sets the controller speed multiplier back to {@link DrivetrainConstants#TRANSLATION_SCALE_NORMAL}.
	 */
	public void resetSpeedMultiplier() {
		setSpeedMultiplier(DrivetrainConstants.TRANSLATION_SCALE_NORMAL);
	}

	/**
	 * Sets the controller speed multiplier. Resets the multiplier when canceled.
	 *
	 * @param speedMultiplier
	 *            Multiplier to set (0, 1].
	 * @return
	 *         Command to run.
	 */
	public Command setSpeedMultiplierCommand(Supplier<Double> speedMultiplier) {
		return Commands.run(() -> setSpeedMultiplier(speedMultiplier.get()))
				.finallyDo(this::resetSpeedMultiplier);
	}

	/**
	 * Drive the robot given a SwerveInputStream, scaled with the {@link #speedMultiplier}.
	 *
	 * @param inputStream
	 *            The {@link SwerveInputStream} to read from.
	 * @param orientation
	 *            The translation's field of reference.
	 * @return
	 *         Command to run.
	 * @see Drivetrain#drive(ChassisSpeeds, TranslationOrientation)
	 */
	private Command driveInputStreamScaledCommand(SwerveInputStream inputStream, TranslationOrientation orientation) {
		return drivetrain.run(() -> {
			inputStream.allianceRelativeControl(orientation == TranslationOrientation.FIELD_RELATIVE);
			inputStream.scaleTranslation(speedMultiplier);
			drivetrain.drive(inputStream.get(), orientation);
		});
	}

	/**
	 * Converts driver input into a SwerveInputStream with default settings.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller.
	 */
	private SwerveInputStream driveGeneric(CommandXboxController controller) {
		return SwerveInputStream.of(drivetrain.getSwerveDrive(), () -> (-1 * controller.getLeftY()), () -> (-1 * controller.getLeftX()))
				.deadband(OperatorConstants.DEADBAND);
	}

	/**
	 * A copy of {@link #driveGeneric(CommandXboxController)} that uses angular velocity control for turning.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller.
	 */
	public SwerveInputStream driveAngularVelocity(CommandXboxController controller) {
		return driveGeneric(controller)
				.withControllerRotationAxis(() -> (-1 * controller.getRightX()));
	}

	/**
	 * A copy of {@link #driveGeneric(CommandXboxController)} that uses heading control for turning.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller.
	 */
	public SwerveInputStream driveDirectAngle(CommandXboxController controller) {
		return driveGeneric(controller)
				.withControllerHeadingAxis(() -> (-1 * controller.getRightX()), () -> (-1 * controller.getRightY()))
				.headingWhile(true);
	}

	/**
	 * A copy of {@link #driveGeneric(CommandXboxController)} that pulls rotation from controller axis 2 for use in simulation.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller.
	 */
	public SwerveInputStream driveDirectAngleSim(CommandXboxController controller) {
		return driveGeneric(controller)
				.withControllerRotationAxis(() -> controller.getRawAxis(2))
				.withControllerHeadingAxis(() -> Math.sin(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2))
				.headingWhile(true);
	}

	/**
	 * A copy of {@link #driveGeneric(CommandXboxController)} that uses a preset heading.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @param heading
	 *            The rotation to point the heading to.
	 * @return
	 *         SwerveInputStream with data from the controller.
	 */
	public SwerveInputStream driveStaticHeading(CommandXboxController controller, Supplier<Rotation2d> heading) {
		return driveGeneric(controller)
				.withControllerHeadingAxis(() -> heading.get().getSin() * (Math.PI * 2), () -> heading.get().getCos() * (Math.PI * 2))
				.headingWhile(true);
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link #driveAngularVelocity(CommandXboxController)}. Calls {@link Drivetrain#resetLastAngleScalar()} on end to prevent snapback.
	 *
	 * @param controller
	 *            Controller to use.
	 * @param orientation
	 *            The translation's field of reference.
	 * @return
	 *         Command to run.
	 */
	public Command driveAngularVelocityCommand(CommandXboxController controller, TranslationOrientation orientation) {
		return driveInputStreamScaledCommand(driveAngularVelocity(controller), orientation)
				.finallyDo(drivetrain::resetLastAngleScalar);
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveDirectAngle(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @param orientation
	 *            The translation's field of reference.
	 * @return
	 *         Command to run.
	 */
	public Command driveDirectAngleCommand(CommandXboxController controller, TranslationOrientation orientation) {
		return driveInputStreamScaledCommand(driveDirectAngle(controller), orientation);
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveDirectAngleSim(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @param orientation
	 *            The translation's field of reference.
	 * @return
	 *         Command to run.
	 */
	public Command driveDirectAngleSimCommand(CommandXboxController controller, TranslationOrientation orientation) {
		return driveInputStreamScaledCommand(driveDirectAngleSim(controller), orientation);
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream, TranslationOrientation)} that uses {@link DrivetrainControls#driveStaticHeading(CommandXboxController, Supplier)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @param orientation
	 *            The translation's field of reference.
	 * @param heading
	 *            The rotation to point the heading to.
	 * @return
	 *         Command to run.
	 */
	public Command driveStaticHeadingCommand(CommandXboxController controller, TranslationOrientation orientation, Supplier<Rotation2d> heading) {
		return driveInputStreamScaledCommand(driveStaticHeading(controller, heading), orientation)
				.finallyDo(() -> drivetrain.setLastAngleScalar(heading));
	}

	/**
	 * {@link #driveStaticHeadingCommand(CommandXboxController, TranslationOrientation, Supplier)} that uses the heading of the nearest pose from a list of poses.
	 *
	 * @param controller
	 *            Controller to use.
	 * @param orientation
	 *            The translation's field of reference.
	 * @param poses
	 *            The list of poses to choose the nearest pose from.
	 * @return
	 *         Command to run.
	 */
	public Command driveStaticHeadingNearestPoseCommand(CommandXboxController controller, TranslationOrientation orientation, List<Pose2d> poses) {
		return driveStaticHeadingCommand(controller, TranslationOrientation.ROBOT_RELATIVE, () -> {
			// Get the rotation of the nearest reef tag
			return drivetrain.getPose()
					.nearest(poses)
					.getRotation();
		});
	}
}
