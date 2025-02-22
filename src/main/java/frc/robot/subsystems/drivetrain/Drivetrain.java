// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.LoggingConstants;
import frc.robot.commands.drivetrain.LockWheelsCommand;
import frc.robot.util.Util;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

/**
 * Subsystem that controls the drivetrain.
 */
public class Drivetrain extends SubsystemBase {
	/**
	 * Swerve drive object.
	 */
	private final SwerveDrive swerveDrive;
	/**
	 * Vision object.
	 */
	private final Vision vision;

	/**
	 * Initialize {@link SwerveDrive} with the directory provided.
	 *
	 * @param directory
	 *            Directory of swerve drive config files.
	 */
	public Drivetrain(File directory) {
		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
		if (LoggingConstants.DEBUG_MODE) {
			SwerveDriveTelemetry.verbosity = DrivetrainConstants.TELEMETRY_VERBOSITY_DEBUG;
		} else {
			SwerveDriveTelemetry.verbosity = DrivetrainConstants.TELEMETRY_VERBOSITY_NORMAL;
		}

		try {
			swerveDrive = new SwerveParser(directory).createSwerveDrive(DrivetrainConstants.MAX_SPEED, DrivetrainConstants.STARTING_POSITION);
			// Alternative method if you don't want to supply the conversion factor via JSON files.
			// swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
		} catch (Exception e) {
			throw new RuntimeException(e);
		}
		swerveDrive.setAutoCenteringModules(false);
		swerveDrive.setHeadingCorrection(DrivetrainConstants.ENABLE_HEADING_CORRECTION);
		swerveDrive.setCosineCompensator(DrivetrainConstants.ENABLE_COSINE_COMPENSATION);// !SwerveDriveTelemetry.isSimulation); // Disables cosine compensation for simulations since it causes discrepancies not seen in real life.
		swerveDrive.setAngularVelocityCompensation(DrivetrainConstants.AngularVelocityCompensation.USE_IN_TELEOP, DrivetrainConstants.AngularVelocityCompensation.USE_IN_AUTO, DrivetrainConstants.AngularVelocityCompensation.ANGULAR_VELOCITY_COEFFICIENT); // Correct for skew that gets worse as angular velocity increases. Start with a coefficient of 0.1.
		swerveDrive.setModuleEncoderAutoSynchronize(DrivetrainConstants.EncoderAutoSynchronization.ENABLED, DrivetrainConstants.EncoderAutoSynchronization.DEADBAND); // Enable if you want to resynchronize your absolute encoders and motor encoders periodically when they are not moving.
		swerveDrive.pushOffsetsToEncoders(); // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible
		swerveDrive.setMotorIdleMode(true);
		// Stop the odometry thread if we are using vision that way we can synchronize updates better.
		// TODO: #112 (Max/Lukas) If we disable vision updates, do we need to restart this thread to ensure odometry is udpated?
		swerveDrive.stopOdometryThread();

		// Set up vision
		vision = new Vision(swerveDrive);
	}

	@Override
	public void periodic() {
		// When vision is enabled we must manually update odometry in SwerveDrive
		// TODO: #106 (MAX/LUKAS) Last year we had a button the drivers could use to disable vision updates
		// if vision was going wonky. It seems that wouldn't work this year as the odometry would no longer
		// be updated?
		if (VisionConstants.ENABLE_VISION) {
			swerveDrive.updateOdometry();
			vision.updatePoseEstimation();
		}
	}

	@Override
	public void simulationPeriodic() {}

	/**
	 * Gets the swerve drive object.
	 *
	 * @return
	 *         {@link SwerveDrive}
	 */
	public SwerveDrive getSwerveDrive() {
		return swerveDrive;
	}

	public Vision getVision() {
		return vision;
	}

	/**
	 * Get the {@link SwerveController} in the swerve drive.
	 *
	 * @return
	 *         {@link SwerveController} from the {@link SwerveDrive}.
	 * @see
	 *      SwerveDrive#swerveController
	 */
	public SwerveController getSwerveController() {
		return swerveDrive.swerveController;
	}

	/**
	 * Get the {@link SwerveDriveConfiguration} object.
	 *
	 * @return
	 *         The {@link SwerveDriveConfiguration} for the current drive.
	 * @see
	 *      SwerveDrive#swerveDriveConfiguration
	 */
	public SwerveDriveConfiguration getSwerveDriveConfiguration() {
		return swerveDrive.swerveDriveConfiguration;
	}

	/**
	 * Get the swerve drive kinematics object.
	 *
	 * @return
	 *         {@link SwerveDriveKinematics} of the swerve drive.
	 * @see
	 *      SwerveDrive#kinematics
	 */
	public SwerveDriveKinematics getKinematics() {
		return swerveDrive.kinematics;
	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose
	 *            The pose to set the odometry to
	 * @see
	 *      SwerveDrive#resetOdometry(Pose2d)
	 */
	public void resetOdometry(Pose2d initialHolonomicPose) {
		swerveDrive.resetOdometry(initialHolonomicPose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by odometry.
	 *
	 * @return
	 *         The robot's pose
	 * @see
	 *      SwerveDrive#getPose()
	 */
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	/**
	 * Command to characterize the robot drive motors using SysId.
	 *
	 * @return
	 *         SysId Drive Command
	 */
	public Command sysIdDriveMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, DrivetrainConstants.SysId.MAX_VOLTS, DrivetrainConstants.SysId.TEST_WITH_SPINNING), DrivetrainConstants.SysId.DELAY, DrivetrainConstants.SysId.QUASI_TIMEOUT, DrivetrainConstants.SysId.DYNAMIC_TIMEOUT);
	}

	/**
	 * Command to characterize the robot angle motors using SysId.
	 *
	 * @return
	 *         SysId Angle Command
	 */
	public Command sysIdAngleMotorCommand() {
		return SwerveDriveTest.generateSysIdCommand(SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), DrivetrainConstants.SysId.DELAY, DrivetrainConstants.SysId.QUASI_TIMEOUT, DrivetrainConstants.SysId.DYNAMIC_TIMEOUT);
	}

	/**
	 * Command to center the modules of the SwerveDrive subsystem.
	 *
	 * @return
	 *         Command to run
	 */
	public Command centerModulesCommand() {
		return Commands.deadline(Commands.waitUntil(() -> (Arrays.asList(swerveDrive.getModules()).stream().allMatch((module) -> (Math.abs(module.getAbsolutePosition()) > 2)))), Commands.run(() -> Arrays.asList(swerveDrive.getModules())
				.forEach(it -> it.setAngle(0.0))));
	}

	/**
	 * Post the trajectory to the field.
	 *
	 * @param trajectory
	 *            The trajectory to post.
	 * @see
	 *      SwerveDrive#postTrajectory(Trajectory)
	 */
	public void postTrajectory(Trajectory trajectory) {
		swerveDrive.postTrajectory(trajectory);
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 *
	 * @see SwerveDrive#zeroGyro()
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
	 *
	 * @see #zeroGyro()
	 */
	public Command zeroGyroCommand() {
		return Commands.runOnce(() -> zeroGyro(), this)
				.ignoringDisable(true);
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drviebase zero command
	 */
	public void zeroGyroWithAlliance() {
		// TODO: (Max) What happens if this is called more than once? Seems like there needs to be logic that it
		// is only called under certain circumstances.
		if (Util.isRedAlliance()) {
			zeroGyro();
			// Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
		} else {
			zeroGyro();
		}
	}

	/**
	 * Sets the drive motors to brake/coast mode.
	 *
	 * @param brake
	 *            True to set motors to brake mode, false for coast.
	 * @see
	 *      SwerveDrive#setMotorIdleMode(boolean)
	 */
	public void setMotorBrake(boolean brake) {
		swerveDrive.setMotorIdleMode(brake);
	}

	/**
	 * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the underlying drivebase.
	 * Note, this is not the raw gyro reading, this may be corrected from calls to resetOdometry().
	 *
	 * @return
	 *         The yaw angle
	 */
	public Rotation2d getHeading() {
		return getPose().getRotation();
	}

	/**
	 * Gets the current field-relative velocity (x, y and omega) of the robot.
	 *
	 * @return
	 *         A ChassisSpeeds object of the current field-relative velocity
	 * @see
	 *      SwerveDrive#getFieldVelocity()
	 */
	public ChassisSpeeds getFieldVelocity() {
		return swerveDrive.getFieldVelocity();
	}

	/**
	 * Gets the current velocity (x, y and omega) of the robot.
	 *
	 * @return
	 *         A {@link ChassisSpeeds} object of the current velocity
	 * @see
	 *      SwerveDrive#getRobotVelocity()
	 */
	public ChassisSpeeds getRobotVelocity() {
		return swerveDrive.getRobotVelocity();
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 *
	 * @see
	 *      SwerveDrive#lockPose()
	 */
	public void lock() {
		swerveDrive.lockPose();
	}

	/**
	 * Command to lock the swerve drive to prevent it from moving.
	 *
	 * @return
	 *         A new {@link LockWheelsCommand}.
	 * @see LockWheelsCommand
	 */
	// TODO: #113 (Max) Only putting in one comment for entire class, but all Commands should start with a capital letter per coding standards. Please update.
	public Command lockCommand() {
		return new LockWheelsCommand(this);
	}

	/**
	 * Gets the current pitch angle of the robot, as reported by the imu.
	 *
	 * @return
	 *         The heading as a {@link Rotation2d} angle
	 * @see
	 *      SwerveDrive#getPitch()
	 */
	public Rotation2d getPitch() {
		return swerveDrive.getPitch();
	}

	/**
	 * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation rate, and
	 * calculates and commands module states accordingly. Can use either open-loop or closed-loop velocity control for
	 * the wheel velocities. Also has field- and robot-relative modes, which affect how the translation vector is used.
	 *
	 * @param translation
	 *            {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
	 *            second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
	 *            torwards port (left). In field-relative mode, positive x is away from the alliance wall
	 *            (field North) and positive y is torwards the left wall when looking through the driver station
	 *            glass (field West).
	 * @param rotation
	 *            Robot angular rate, in radians per second. CCW positive. Unaffected by field/robot
	 *            relativity.
	 * @param fieldRelative
	 *            Drive mode. True for field-relative, false for robot-relative.
	 * @see
	 *      SwerveDrive#drive(Translation2d, double, boolean, boolean)
	 */
	public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
		swerveDrive.drive(translation, rotation, fieldRelative, false); // Open loop is disabled since it shouldn't be used most of the time.
	}

	// TODO: #115 (Max) Please simplify this code. There are 12 functions/commands related to resetting the
	// lastAngleScalar.
	/**
	 * Sets the angle that heading control will return to if no angle is inputed.
	 *
	 * @param angle
	 *            Angle to set in radians.
	 * @see SwerveController#lastAngleScalar
	 * @implNote
	 *           Running this on the Red alliance will cause the angle to be flipped 180 degrees. Call {@link #setLastAngleScalarByAlliance(Rotation2d)} instead if that is not the desired behavior.
	 */
	// TODO: #114 (Max) Shouldn't this be private to ensure that the wrong function isn't used? Should always use ByAlliance version
	public void setLastAngleScalar(double angle) {
		swerveDrive.swerveController.lastAngleScalar = angle;
	}

	/**
	 * Sets the angle that heading control will return to if no angle is inputed.
	 *
	 * @param rotation
	 *            Angle to set.
	 * @see SwerveController#lastAngleScalar
	 * @implNote
	 *           Running this on the Red alliance will cause the angle to be flipped 180 degrees. Call {@link #setLastAngleScalarByAlliance(Rotation2d)} instead if that is not the desired behavior.
	 */
	// TODO: (Max) Shouldn't this be private to ensure that the wrong function isn't used? Should always use ByAlliance version
	public void setLastAngleScalar(Rotation2d rotation) {
		swerveDrive.swerveController.lastAngleScalar = rotation.getRadians();
	}

	/**
	 * Sets the angle that heading control will return to if no angle is inputed.
	 *
	 * @param rotation
	 *            Angle to set. Rotated by 180 degrees.
	 * @see SwerveController#lastAngleScalar
	 * @implNote
	 *           Running this on the Blue alliance will cause the angle to be flipped 180 degrees. Call {@link #setLastAngleScalarByAlliance(Rotation2d)} instead if that is not the desired behavior.
	 */
	// TODO: (Max) Shouldn't this be private to ensure that the wrong function isn't used? Should always use ByAlliance version
	// TODO: #116 (Max) Have you tested this works?
	public void setLastAngleScalarInverted(Rotation2d rotation) {
		swerveDrive.swerveController.lastAngleScalar = rotation.rotateBy(Rotation2d.k180deg).getRadians();
	}

	/**
	 * Sets the angle that heading control will turn to if no angle is inputed. Inverted based on alliance color.
	 *
	 * @param rotation
	 *            Angle to set.
	 * @see #setLastAngleScalar(Rotation2d)
	 * @see #setLastAngleScalarInverted(Rotation2d)
	 */
	public void setLastAngleScalarByAlliance(Rotation2d rotation) {
		if (Util.isRedAlliance()) {
			setLastAngleScalarInverted(rotation);
		} else {
			setLastAngleScalar(rotation);
		}
	}

	/**
	 * Sets the angle that heading control will turn to if no angle is inputed. Inverted based on alliance color.
	 *
	 * @param rotation
	 *            Angle to set.
	 * @see #setLastAngleScalar(Rotation2d)
	 * @see #setLastAngleScalarInverted(Rotation2d)
	 */
	public void setLastAngleScalarByAlliance(Supplier<Rotation2d> rotation) {
		setLastAngleScalarByAlliance(rotation.get());
	}

	/**
	 * Command to set the angle that heading control will turn to if no angle is inputed. Inverted based on alliance color.
	 *
	 * @param rotation
	 *            Angle to set.
	 * @return
	 *         {@link Command} to run.
	 * @see #setLastAngleScalarByAlliance(Rotation2d)
	 */
	public Command setLastAngleScalarByAllianceCommand(Supplier<Rotation2d> rotation) {
		return Commands.runOnce(() -> setLastAngleScalarByAlliance(rotation));
	}

	/**
	 * The method to reset what the heading control will turn to if no angle is inputed. Used to prevent angle snapback.
	 *
	 * @see #setLastAngleScalar(Rotation2d)
	 * @implNote
	 *           Running this on the Red alliance will cause the robot to flip 180 degrees. Call {@link #resetLastAngleScalarByAlliance()} instead if that is not the desired behavior.
	 */
	public void resetLastAngleScalar() {
		setLastAngleScalar(getHeading());
	}

	/**
	 * The command to reset what the heading control will turn to if no angle is inputed. Used to prevent angle snapback.
	 *
	 * @return
	 *         {@link Command} to run.
	 * @see #resetLastAngleScalar()
	 */
	public Command resetLastAngleScalarCommand() {
		return Commands.runOnce(() -> resetLastAngleScalar(), this);
	}

	/**
	 * Inverted method to reset what the heading control will turn to if no angle is inputed. Used to prevent angle snapback.
	 *
	 * @see #resetLastAngleScalar()
	 * @see #setLastAngleScalarInverted(Rotation2d)
	 * @implNote
	 *           Running this on the Blue alliance will cause the robot to flip 180 degrees. Call {@link #resetLastAngleScalarByAlliance()} instead if that is not the desired behavior.
	 */
	public void resetLastAngleScalarInverted() {
		setLastAngleScalarInverted(getHeading());
	}

	/**
	 * Inverted command to reset what the heading control will turn to if no angle is inputed. Used to prevent angle snapback.
	 *
	 * @return
	 *         {@link Command} to run.
	 * @see #resetLastAngleScalarInverted()
	 */
	public Command resetLastAngleScalarInvertedCommand() {
		return Commands.runOnce(() -> resetLastAngleScalarInverted(), this);
	}

	/**
	 * Method to reset what the heading control will turn to if no angle is inputed. Inverted based on alliance color. Used to prevent angle snapback.
	 *
	 * @see #resetLastAngleScalar()
	 * @see #resetLastAngleScalarInverted()
	 */
	public void resetLastAngleScalarByAlliance() {
		if (Util.isRedAlliance()) {
			resetLastAngleScalarInverted();
		} else {
			resetLastAngleScalar();
		}
	}

	/**
	 * Command to reset what the heading control will turn to if no angle is inputed. Inverted based on alliance color. Used to prevent angle snapback.
	 *
	 * @return
	 *         {@link Command} to run.
	 * @see #resetLastAngleScalarCommand()
	 * @see #resetLastAngleScalarInvertedCommand()
	 */
	public Command resetLastAngleScalarByAllianceCommand() {
		return Commands.either(resetLastAngleScalarInvertedCommand(), resetLastAngleScalarCommand(), () -> Util.isRedAlliance());
	}

	/**
	 * Use PathPlanner Path finding to go to a point on the field.
	 *
	 * @param pose
	 *            Target {@link Pose2d} to go to.
	 * @return
	 *         PathFinding command
	 * @see AutoBuilder#pathfindToPose(Pose2d, PathConstraints, edu.wpi.first.units.measure.LinearVelocity)
	 */
	public Command driveToPoseCommand(Supplier<Pose2d> pose) {
		// Create the constraints to use while pathfinding
		PathConstraints constraints = new PathConstraints(MetersPerSecond.of(swerveDrive.getMaximumChassisVelocity()), DrivetrainConstants.Pathfinding.MAX_LINEAR_ACCELERATION, RadiansPerSecond.of(swerveDrive.getMaximumChassisAngularVelocity()), DrivetrainConstants.Pathfinding.MAX_ANGULAR_ACCELERATION);

		// Since AutoBuilder is configured, we can use it to build pathfinding commands
		return Commands.defer(() -> AutoBuilder.pathfindToPose(pose.get(), constraints, MetersPerSecond.of(0) // Goal end velocity in meters/sec
		), new HashSet<Subsystem>(Set.of(this)))
				.andThen(setLastAngleScalarByAllianceCommand(() -> pose.get().getRotation()))
				.handleInterrupt(this::resetLastAngleScalarByAlliance);
	}

	/**
	 * Drives to the nearest pose out of a list.
	 *
	 * @param poseList
	 *            List of poses to choose from.
	 * @return
	 *         {@link Command} to run.
	 */
	public Command driveToNearestPoseCommand(List<Pose2d> poseList) {
		return driveToPoseCommand(() -> getPose().nearest(poseList));
	}

	/**
	 * Drive with {@link SwerveSetpointGenerator} from 254, implemented by PathPlanner.
	 *
	 * @param robotRelativeChassisSpeed
	 *            Robot relative {@link ChassisSpeeds} to achieve.
	 * @return
	 *         {@link Command} to run.
	 * @throws IOException
	 *             If the PathPlanner GUI settings is invalid
	 * @throws ParseException
	 *             If PathPlanner GUI settings is nonexistent.
	 */
	private Command driveWithSetpointGeneratorCommand(Supplier<ChassisSpeeds> robotRelativeChassisSpeed) throws IOException, ParseException {
		SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(), swerveDrive.getMaximumChassisAngularVelocity());
		AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(new SwerveSetpoint(swerveDrive.getRobotVelocity(), swerveDrive.getStates(), DriveFeedforwards.zeros(swerveDrive.getModules().length)));
		AtomicReference<Double> previousTime = new AtomicReference<>();

		return startRun(() -> previousTime.set(Timer.getFPGATimestamp()), () -> {
			double newTime = Timer.getFPGATimestamp();
			SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(), robotRelativeChassisSpeed.get(), newTime - previousTime.get());
			swerveDrive.drive(newSetpoint.robotRelativeSpeeds(), newSetpoint.moduleStates(), newSetpoint.feedforwards().linearForces());
			prevSetpoint.set(newSetpoint);
			previousTime.set(newTime);
		});
	}

	/**
	 * Drive with 254's Setpoint generator; port written by PathPlanner.
	 *
	 * @param fieldRelativeSpeeds
	 *            Field-Relative {@link ChassisSpeeds}
	 * @return
	 *         Command to drive the robot using the setpoint generator.
	 * @see
	 *      #driveWithSetpointGeneratorCommand(Supplier)
	 */
	public Command driveWithSetpointGeneratorFieldRelativeCommand(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
		try {
			return driveWithSetpointGeneratorCommand(() -> {
				return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), getHeading());
			});
		} catch (Exception e) {
			DriverStation.reportError(e.toString(), true);
		}
		return Commands.none();
	}

	/**
	 * Command that drives the swerve drive to a specific distance at a given speed.
	 *
	 * @param distanceInMeters
	 *            the distance to drive in meters
	 * @param speedInMetersPerSecond
	 *            the speed at which to drive in meters per second
	 * @return
	 *         Command to run
	 */
	public Command driveToDistanceCommand(double distanceInMeters, double speedInMetersPerSecond) {
		return run(() -> drive(new ChassisSpeeds(speedInMetersPerSecond, 0, 0)))
				.until(() -> swerveDrive.getPose().getTranslation().getDistance(Translation2d.kZero) > distanceInMeters);
	}

	/**
	 * Command to drive the robot using translative values and heading as angular velocity.
	 *
	 * @param translationX
	 *            Translation in the X direction. Cubed for smoother controls.
	 * @param translationY
	 *            Translation in the Y direction. Cubed for smoother controls.
	 * @param angularRotationX
	 *            Angular velocity of the robot to set. Cubed for smoother controls.
	 * @return
	 *         Command to run
	 * @see
	 *      SwerveDrive#drive(Translation2d, double, boolean, boolean)
	 */
	public Command driveAngularCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX) {
		return run(() -> {
			// Make the robot move
			swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(), translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), DrivetrainConstants.TRANSLATION_SCALE_NORMAL), Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(), true, false);
		});
	}

	/**
	 * Command to drive the robot using translative values and heading as a setpoint.
	 *
	 * @param translationX
	 *            Translation in the X direction. Cubed for smoother controls.
	 * @param translationY
	 *            Translation in the Y direction. Cubed for smoother controls.
	 * @param headingX
	 *            Heading X to calculate angle of the joystick.
	 * @param headingY
	 *            Heading Y to calculate angle of the joystick.
	 * @return
	 *         Command to run
	 */
	public Command driveHeadingCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX, DoubleSupplier headingY) {
		// swerveDrive.setHeadingCorrection(true); // Normally you would want heading correction for this kind of control.
		return run(() -> {
			Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(), translationY.getAsDouble()), DrivetrainConstants.TRANSLATION_SCALE_NORMAL);

			// Make the robot move
			driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(), headingX.getAsDouble(), headingY.getAsDouble(), swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumChassisVelocity()));
		});
	}

	/**
	 * Drive according to the chassis robot oriented velocity.
	 *
	 * @param velocity
	 *            Robot oriented {@link ChassisSpeeds}
	 * @see
	 *      SwerveDrive#drive(ChassisSpeeds)
	 */
	public void drive(ChassisSpeeds velocity) {
		swerveDrive.drive(velocity);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 * @see
	 *      SwerveDrive#driveFieldOriented(ChassisSpeeds)
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		swerveDrive.driveFieldOriented(velocity);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity
	 *            Velocity according to the field.
	 * @return
	 *         Command to run
	 * @see
	 *      SwerveDrive#driveFieldOriented(ChassisSpeeds)
	 */
	public Command driveFieldOrientedCommand(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		});
	}

	/**
	 * {@link #driveFieldOrientedCommand(Supplier)} that uses {@link DrivetrainControls#driveAngularVelocity(Drivetrain, CommandXboxController)}. Calls {@link #resetLastAngleScalar()} on end to prevent snapback.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedAngularVelocityControllerCommand(CommandXboxController controller) {
		return driveFieldOrientedCommand(DrivetrainControls.driveAngularVelocity(this, controller))
				.finallyDo(this::resetLastAngleScalarByAlliance);
	}

	/**
	 * {@link #driveFieldOrientedCommand(Supplier)} that uses {@link DrivetrainControls#driveDirectAngle(Drivetrain, CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleControllerCommand(CommandXboxController controller) {
		return driveFieldOrientedCommand(DrivetrainControls.driveDirectAngle(this, controller));
	}

	/**
	 * {@link #driveFieldOrientedCommand(Supplier)} that uses {@link DrivetrainControls#driveDirectAngleSim(Drivetrain, CommandXboxController)}.
	 *
	 * @param controller
	 *            Controller to use.
	 * @return
	 *         Command to run.
	 */
	public Command driveFieldOrientedDirectAngleSimControllerCommand(CommandXboxController controller) {
		return driveFieldOrientedCommand(DrivetrainControls.driveDirectAngleSim(this, controller));
	}

	/**
	 * Set chassis speeds with closed-loop velocity control.
	 *
	 * @param chassisSpeeds
	 *            Chassis Speeds to set.
	 * @see
	 *      SwerveDrive#setChassisSpeeds(ChassisSpeeds)
	 */
	public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
		swerveDrive.setChassisSpeeds(chassisSpeeds);
	}
}
