package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
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
	 * @return
	 *         Command to run.
	 * @see Drivetrain#driveFieldOriented(ChassisSpeeds)
	 */
	private Command driveInputStreamScaledCommand(SwerveInputStream inputStream) {
		return drivetrain.run(() -> {
			inputStream.scaleTranslation(speedMultiplier);
			drivetrain.drive(inputStream.get(), true);
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
				.withControllerRotationAxis(() -> (-1 * controller.getRightX()))
				.allianceRelativeControl(true);
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
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream)} that uses {@link #driveAngularVelocity(CommandXboxController)}. Calls {@link Drivetrain#resetLastAngleScalar()} on end to prevent snapback.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedAngularVelocityCommand(CommandXboxController controller) {
		return driveInputStreamScaledCommand(driveAngularVelocity(controller))
				.finallyDo(drivetrain::resetLastAngleScalar);
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream)} that uses {@link DrivetrainControls#driveDirectAngle(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleCommand(CommandXboxController controller) {
		return driveInputStreamScaledCommand(driveDirectAngle(controller));
	}

	/**
	 * {@link #driveInputStreamScaledCommand(SwerveInputStream)} that uses {@link DrivetrainControls#driveDirectAngleSim(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleSimCommand(CommandXboxController controller) {
		return driveInputStreamScaledCommand(driveDirectAngleSim(controller));
	}
}
