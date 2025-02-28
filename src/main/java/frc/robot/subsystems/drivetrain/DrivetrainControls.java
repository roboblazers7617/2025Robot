package frc.robot.subsystems.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveInputStream;

/**
 * Class that contains utility functions for controlling the {@link Drivetrain} with HID controllers.
 */
public final class DrivetrainControls {
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
		return SwerveInputStream.of(drivetrain.getSwerveDrive(), () -> (-1 * controller.getLeftY()), () -> (-1 * controller.getLeftX()))
				.withControllerRotationAxis(() -> (-1 * controller.getRightX()))
				.deadband(OperatorConstants.DEADBAND)
				.scaleTranslation(DrivetrainConstants.TRANSLATION_SCALE_NORMAL)
				.allianceRelativeControl(true);
	}

	/**
	 * A copy of {@link #driveAngularVelocity(Drivetrain, CommandXboxController)} that uses heading control.
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
				.withControllerHeadingAxis(() -> (-1 * controller.getRightX()), () -> (-1 * controller.getRightY()))
				.headingWhile(true);
	}

	/**
	 * A copy of {@link #driveAngularVelocity(Drivetrain, CommandXboxController)} that uses aim control.
	 *
	 * @param drivetrain
	 *            the drivetrain to control
	 * @param controller
	 *            the controller to read from
	 * @param pose
	 *            the pose to aim at
	 * @return
	 *         SwerveInputStream with data from the controller
	 */
	public static SwerveInputStream driveAim(Drivetrain drivetrain, CommandXboxController controller, Supplier<Pose2d> pose) {
		return driveAngularVelocity(drivetrain, controller)
				.aim(pose.get())
				.aimWhile(true);
	}

	/**
	 * A copy of {@link #driveAngularVelocity(Drivetrain, CommandXboxController)} that pulls rotation from controller axis 2 for use in simulation.
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
