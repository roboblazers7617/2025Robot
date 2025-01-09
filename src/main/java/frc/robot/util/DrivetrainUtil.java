package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Drivetrain;
import swervelib.SwerveInputStream;

public final class DrivetrainUtil {
	/**
	 * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
	 *
	 * @param drivetrain
	 *            the drivetrain to control
	 * @param controller
	 *            the controller to read from
	 * @return
	 *         SwerveInputStream with data from the controller
	 */
	public static SwerveInputStream driveAngularVelocity(Drivetrain drivetrain, CommandXboxController controller) {
		return SwerveInputStream.of(drivetrain.getSwerveDrive(), () -> controller.getLeftY() * -1, () -> controller.getLeftX() * -1)
				.withControllerRotationAxis(controller::getRightX)
				.deadband(OperatorConstants.DEADBAND)
				.scaleTranslation(0.8)
				.allianceRelativeControl(true);
	}

	/**
	 * Clones the angular velocity input stream and converts it to a fieldRelative input stream.
	 *
	 * @param drivetrain
	 *            the drivetrain to control
	 * @param controller
	 *            the controller to read from
	 * @return
	 *         SwerveInputStream with data from the controller
	 */
	public static SwerveInputStream driveDirectAngle(Drivetrain drivetrain, CommandXboxController controller) {
		return driveAngularVelocity(drivetrain, controller)
				.withControllerHeadingAxis(controller::getRightX, controller::getRightY)
				.headingWhile(true);
	}

	/**
	 * A copy of {@link driveDirectAngle} for use in simulation.
	 *
	 * @param drivetrain
	 *            the drivetrain to control
	 * @param controller
	 *            the controller to read from
	 * @return
	 *         SwerveInputStream with data from the controller
	 */
	public static SwerveInputStream driveDirectAngleSim(Drivetrain drivetrain, CommandXboxController controller) {
		return driveAngularVelocity(drivetrain, controller)
				.withControllerRotationAxis(() -> controller.getRawAxis(2))
				.withControllerHeadingAxis(() -> Math.sin(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2), () -> Math.cos(controller.getRawAxis(2) * Math.PI) * (Math.PI * 2))
				.headingWhile(true);
	}
}
