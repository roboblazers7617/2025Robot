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
	 * Speed multiplier used for scaling controller inputs.
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
	 *            Multiplier to set.
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
	 *            Multiplier to set.
	 * @return
	 *         Command to run.
	 */
	public Command setSpeedMultiplierCommand(Supplier<Double> speedMultiplier) {
		return Commands.run(() -> setSpeedMultiplier(speedMultiplier.get()))
				.finallyDo(this::resetSpeedMultiplier);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity, scaled with the {@link #speedMultiplier}.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 * @see swervelib.SwerveDrive#driveFieldOriented(ChassisSpeeds)
	 */
	private void driveFieldOrientedScaled(ChassisSpeeds velocity) {
		drivetrain.driveFieldOriented(velocity.times(speedMultiplier));
	}

	/**
	 * Drive the robot given a chassis field oriented velocity, scaled with the {@link #speedMultiplier}.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 * @return
	 *         Command to run
	 * @see swervelib.SwerveDrive#driveFieldOriented(ChassisSpeeds)
	 */
	private Command driveFieldOrientedScaledCommand(Supplier<ChassisSpeeds> velocity) {
		return drivetrain.run(() -> {
			driveFieldOrientedScaled(velocity.get());
		});
	}

	/**
	 * Converts driver input into a SwerveInputStream with default settings.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller
	 */
	private SwerveInputStream driveGeneric(CommandXboxController controller) {
		return SwerveInputStream.of(drivetrain.getSwerveDrive(), () -> (-1 * controller.getLeftY()), () -> (-1 * controller.getLeftX()))
				.deadband(OperatorConstants.DEADBAND)
				.scaleTranslation(DrivetrainConstants.TRANSLATION_SCALE_NORMAL);
	}

	/**
	 * A copy of {@link #driveGeneric(CommandXboxController)} that uses angular velocity control for turning.
	 *
	 * @param controller
	 *            The controller to read from.
	 * @return
	 *         SwerveInputStream with data from the controller
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
	 *         SwerveInputStream with data from the controller
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
	 *         SwerveInputStream with data from the controller
	 */
	public SwerveInputStream driveDirectAngleSim(CommandXboxController controller) {
		return driveGeneric(controller)
				.withControllerRotationAxis(() -> controller.getRawAxis(2))
				.withControllerHeadingAxis(() -> Math.sin(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2))
				.headingWhile(true);
	}

	/**
	 * {@link #driveFieldOrientedScaledCommand(Supplier)} that uses {@link #driveAngularVelocity(CommandXboxController)}. Calls {@link Drivetrain#resetLastAngleScalar()} on end to prevent snapback.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedAngularVelocityCommand(CommandXboxController controller) {
		return driveFieldOrientedScaledCommand(driveAngularVelocity(controller))
				.finallyDo(drivetrain::resetLastAngleScalar);
	}

	/**
	 * {@link #driveFieldOrientedScaledCommand(Supplier)} that uses {@link DrivetrainControls#driveDirectAngle(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleCommand(CommandXboxController controller) {
		return driveFieldOrientedScaledCommand(driveDirectAngle(controller));
	}

	/**
	 * {@link #driveFieldOrientedScaledCommand(Supplier)} that uses {@link DrivetrainControls#driveDirectAngleSim(CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleSimCommand(CommandXboxController controller) {
		return driveFieldOrientedScaledCommand(driveDirectAngleSim(controller));
	}
}
