// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import java.io.File;
import java.util.List;
import java.util.stream.IntStream;
import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.PoseUtil;
import io.github.roboblazers7617.limelight.LimelightSettings.ImuMode;
import io.github.roboblazers7617.limelight.PoseEstimator.PoseEstimators;
import swervelib.math.Matter;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.LinearAcceleration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
	/**
	 * Constants that contain physical information about the robot.
	 */
	public static class PhysicalConstants {
		/**
		 * Mass of the robot in kilograms.
		 */
		// TODO: #140 (Max) Need to update with actual weight of robot
		public static final double ROBOT_MASS = Pounds.of(100).in(Kilograms);
		/**
		 * Matter representing the robot chassis.
		 */
		// TODO: #141 Need to udpate with actual COG
		public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);
	}

	/**
	 * Constants used by the {@link frc.robot.subsystems.drivetrain.Drivetrain}.
	 */
	public static class DrivetrainConstants {
		/**
		 * Maximum speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = FeetPerSecond.of(14.5).in(MetersPerSecond);
		/**
		 * Directory that contains the YAGSL configuration.
		 */
		public static final File CONFIG_DIR = new File(Filesystem.getDeployDirectory(), "swerve");
		/**
		 * YAGSL telemetry verbosity when in debug mode.
		 */
		public static final TelemetryVerbosity TELEMETRY_VERBOSITY_DEBUG = TelemetryVerbosity.HIGH;
		/**
		 * YAGSL telemetry verbosity when in normal mode.
		 */
		public static final TelemetryVerbosity TELEMETRY_VERBOSITY_NORMAL = TelemetryVerbosity.POSE;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in fast mode.
		 */
		public static final double TRANSLATION_SCALE_FAST = 1;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in normal mode.
		 */
		public static final double TRANSLATION_SCALE_NORMAL = 0.8;
		/**
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain in slow mode.
		 */
		public static final double TRANSLATION_SCALE_SLOW = 0.6;
		/**
		 * Starting pose.
		 */
		public static final Pose2d STARTING_POSITION = new Pose2d(new Translation2d(Meters.of(1), Meters.of(4)), Rotation2d.fromDegrees(0));
		/**
		 * Enables {@link swervelib.SwerveDrive#headingCorrection heading correction}. Should only be used while controlling the robot via angle.
		 */
		public static final boolean ENABLE_HEADING_CORRECTION = false;
		/**
		 * Enables {@link swervelib.parser.SwerveModuleConfiguration#useCosineCompensator cosine compensation}.
		 */
		public static final boolean ENABLE_COSINE_COMPENSATION = false;

		/**
		 * Angular velocity skew correction configuration.
		 *
		 * @see swervelib.SwerveDrive#setAngularVelocityCompensation
		 */
		public static final class AngularVelocityCompensation {
			/**
			 * Enables angular velocity correction in teleop.
			 */
			public static final boolean USE_IN_TELEOP = true;
			/**
			 * Enables angular velocity correction in autonomous.
			 */
			public static final boolean USE_IN_AUTO = true;
			/**
			 * The angular velocity coefficient.
			 */
			public static final double ANGULAR_VELOCITY_COEFFICIENT = 0.1;
		}

		/**
		 * Configure auto synchronization for encoders during a match.
		 *
		 * @see swervelib.SwerveDrive#setModuleEncoderAutoSynchronize
		 */
		public static final class EncoderAutoSynchronization {
			/**
			 * Enable auto synchronization.
			 */
			public static final boolean ENABLED = false;
			/**
			 * Deadband in degrees.
			 */
			public static final double DEADBAND = 1;
		}

		/**
		 * Configure pathfinding to poses.
		 */
		public static final class Pathfinding {
			/**
			 * Maximium linear acceleration.
			 */
			public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(2.0);
			/**
			 * Maximum angular acceleration.
			 */
			public static final AngularAcceleration MAX_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(720);
		}

		/**
		 * SysId configuration.
		 */
		public static final class SysId {
			/**
			 * The maximum voltage to apply to the drive motors.
			 */
			public static final double MAX_VOLTS = 12.0;
			/**
			 * Delay in seconds between each section. This time allows for things to settle (allow motors to spin down, etc.).
			 */
			public static final double DELAY = 3.0;
			/**
			 * Time in seconds to run Quasistatic routines. This prevents the robot from going too far.
			 */
			public static final double QUASI_TIMEOUT = 5.0;
			/**
			 * Time in seconds to run Dynamic routines.
			 */
			public static final double DYNAMIC_TIMEOUT = 3.0;
			/**
			 * Spin in place instead of driving forward.
			 */
			public static final boolean TEST_WITH_SPINNING = false;
		}
	}

	/**
	 * Constants used to configure the operator controllers.
	 */
	public static class OperatorConstants {
		/**
		 * Controller port index where the driver controller is connected.
		 */
		public static final int DRIVER_CONTROLLER_PORT = 0;
		/**
		 * Controller port index where the operator controller is connected.
		 */
		public static final int OPERATOR_CONTROLLER_PORT = 1;
		/**
		 * Joystick deadband.
		 */
		public static final double DEADBAND = 0.1;

		/**
		 * Type of game piece for the robot to interact with.
		 */
		public static enum GamepieceMode {
			/**
			 * To interact with Coral
			 */
			CORAL_MODE,
			/**
			 * To interact with Algae
			 */
			ALGAE_MODE
		}
	}

	/**
	 * Constants used to configure the autonomous program.
	 */
	public static class AutoConstants {
		/**
		 * PID constants used for translation.
		 */
		public static final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
		/**
		 * PID constants used for rotation.
		 */
		public static final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(5.0, 0.0, 0.0);
	}

	// TODO: #98 (Brandon) Add documentation here on what settings should be during comp versus testing
	public static class LoggingConstants {
		/**
		 * Send logging data to NetworkTables. Data is written to storage when set to false.
		 */
		public static final boolean DEBUG_MODE = true;
		/**
		 * Log all data above specified level.
		 */
		public static final Logged.Importance DEBUG_LEVEL = Logged.Importance.DEBUG;
	}

	/**
	 * Constants used to configure the driver dashboard.
	 */
	public static class DashboardConstants {
		/**
		 * The name of the tab used in Auto.
		 */
		public static final String AUTO_TAB_NAME = "Autonomous";
		/**
		 * The name of the tab used in Teleop.
		 */
		public static final String TELEOP_TAB_NAME = "Teleoperated";
	}

	/**
	 * Constants used to configure vision.
	 */
	public static class VisionConstants {
		/**
		 * The name of the front Limelight on NetworkTables.
		 */
		public static final String FRONT_LIMELIGHT_NAME = "limelight-front";
		/**
		 * The name of the back Limelight on NetworkTables.
		 */
		public static final String BACK_LIMELIGHT_NAME = "limelight-back";
		/**
		 * Enable vision odometry updates.
		 */
		public static final boolean ENABLE_VISION = true;
		/**
		 * Use MegaTag2 for pose estimation.
		 */
		public static final PoseEstimators POSE_ESTIMATOR_TYPE = PoseEstimators.BLUE_MEGATAG2;
		/**
		 * The frequency of processed vision frames while disabled.
		 */
		public static final int DISABLED_UPDATE_FREQUENCY = 60;
		/**
		 * The {@link ImuMode} to use while disabled.
		 */
		public static final ImuMode DISABLED_IMU_MODE = ImuMode.SyncInternalImu;
		/**
		 * The {@link ImuMode} to use while enabled.
		 */
		public static final ImuMode ENABLED_IMU_MODE = ImuMode.ExternalAssistInternalIMU;
	}

	/**
	 * Constants used to configure the autonomous program.
	 */
	public static class ClimberConstants {
		/**
		 * Port for the right climber motor.
		 */
		// TODO: make sure these get set
		// TODO: (Sam) Please update with correct values
		public static final int RIGHT_CLIMBER_PORT = 0;
		/**
		 * Port for the left climber motor.
		 */
		// TODO: (Sam) Please update with correct values
		public static final int LEFT_CLIMBER_PORT = 1;
	}

	/**
	 * Constants used to configure the {@link frc.robot.subsystems.endEffector.EndEffector End Effector}.
	 */
	public static class EndEffectorConstants {
		/**
		 * End Effector's motor gear ratio.
		 */
		public static final double GEAR_RATIO_END_EFFECTOR_MOTOR = (2.5 / 1.0);
		/**
		 * Neo's current limit.
		 */
		public static final int MAX_CURRENT_LIMIT = 40;
		/**
		 * End Effector's Spark Max CAN ID.
		 */
		public static final int CAN_ID_END_EFFECTOR = 41;
		/*
		 * I Belive we used this to make the numbers apear correctly in the dirvers station?
		 */
		public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO_END_EFFECTOR_MOTOR * 360.0;
		/**
		 * PID config for the motor controller.
		 */
		// TODO: PID values block to be updated with actual values
		public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
				.p(0)
				.i(0)
				.d(0);
		/**
		 * DIO pin for the beam break.
		 */
		public static final int BEAM_BREAK_DIO = 0;
		/**
		 * Intake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_INTAKE_SPEED = 0.7;
		/**
		 * Outtake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_OUTAKE_SPEED = 0.2;
		/**
		 * Intake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_INTAKE_SPEED = -0.2;
		/**
		 * Outtake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_OUTAKE_SPEED = 0.5;
		/**
		 * Time (in seconds) that the motors run after beam break detects no coral after using the outtake command
		 */
		public static final double OUTTAKE_WAIT_TIME = 0.2;
		/**
		 * Time (in seconds) that the motors run after algae outake is called to eject algae.
		 */
		public static final double ALGAE_OUTTAKE_RUN_TIME = 0.3;
		/**
		 * Time (in seconds) determines how long the break period is before the Current spike can be detected.
		 * This allows it to not shutoff with the initial motor startup spike. (this will need to be adjusted)
		 */
		public static final double MOTOR_CURRENT_CHECK_DELAY = 0.1;
		/**
		 * Limit to the current before it shuts off the motor for the Algae Intake system. (also needs to be adjusted)
		 */
		public static final double AlGAE_INTAKE_CURRENT_SHUTOFF_THRESHOLD = 25.0;
		/**
		 * Alternate method to shutof algae intake method using a minimum motor speed limit.
		 */
		public static final double ALGAE_INTAKE_MINIMUM_SHUTOFF_SPEED = -0.05;
	}

	/**
	 * Constants for the Ramp Motor
	 */
	public static class RampConstants {
		/**
		 * Ramp gear ratio
		 */
		// TODO: set proper gear ratio
		public static final double RAMP_MOTOR_GEAR_RATIO = (1.0 / 60.0);
		/**
		 * Ramp CAN_ID
		 */
		// TODO: set ramp CAN_ID once assigned
		public static final int RAMP_MOTOR_CAN_ID = 32;
		/**
		 * PID config for the motor controller.
		 */
		// TODO: PID values block to be updated with actual values
		public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
				.p(0.02)
				.i(0)
				.d(0);
		/**
		 * Ramp motor's current limit
		 */
		public static final int RAMP_MOTOR_CURRENT_LIMIT = 20;
		/*
		 * I Belive we used this to make the numbers apear correctly in the dirvers station?
		 * may be a usesless holdover from arm code test though
		 */
		public static final double POSITION_CONVERSION_FACTOR = RAMP_MOTOR_GEAR_RATIO * 360.0;
		/**
		 * Ramp Stow Position
		 */
		// TODO: Update with final value
		public static final double RAMP_STOW_POSITION = 75.0;
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

		/**
		 * Constants relating to the reef.
		 */
		// TODO: #101 (Max) This will work for moving to score a coral. How do you move to remove an algae?
		public static class Reef {
			/**
			 * AprilTag IDs for the reef on the blue alliance.
			 */
			public static final List<Integer> TAG_IDS = new ArrayList<Integer>(List.of(17, 18, 19, 20, 21, 22));
			/**
			 * AprilTag poses for the reef on the blue alliance.
			 */
			public static final List<Pose3d> TAG_POSES = new ArrayList<Pose3d>();
			/**
			 * Offset from the AprilTag from which coral scoring should happen.
			 */
			public static final Transform2d CORAL_SCORING_OFFSET = new Transform2d(Meters.of(0.4), Meters.of(0.33 / 2), Rotation2d.k180deg);
			/**
			 * Offset from the AprilTag from which algae scoring should happen.
			 */
			public static final Transform2d ALGAE_SCORING_OFFSET = new Transform2d(Meters.of(0.5), Meters.of(0), Rotation2d.k180deg);
			/**
			 * Poses from which the robot can score coral on the left side on the blue alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_BLUE_LEFT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral on the right side on the blue alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_BLUE_RIGHT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral on the left side on the red alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_RED_LEFT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral on the right side on the red alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_RED_RIGHT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score algae on the blue alliance.
			 */
			public static final List<Pose2d> ALGAE_SCORING_POSES_BLUE = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score algae on the red alliance.
			 */
			public static final List<Pose2d> ALGAE_SCORING_POSES_RED = new ArrayList<Pose2d>();

			static {
				// Generate a list of tag poses.
				TAG_IDS.forEach((id) -> {
					Optional<Pose3d> tagPose = FIELD_LAYOUT.getTagPose(id);

					if (tagPose.isPresent()) {
						TAG_POSES.add(tagPose.get());
					}
				});

				// Generate lists of coral scoring poses.
				TAG_POSES.forEach((pose) -> {
					Pose2d pose2d = pose.toPose2d();
					// Regular side
					CORAL_SCORING_POSES_BLUE_RIGHT.add(pose2d.transformBy(CORAL_SCORING_OFFSET));
					// Flipped side
					CORAL_SCORING_POSES_BLUE_LEFT.add(pose2d.transformBy(new Transform2d(CORAL_SCORING_OFFSET.getMeasureX(), CORAL_SCORING_OFFSET.getMeasureY().times(-1), CORAL_SCORING_OFFSET.getRotation())));
				});

				// Generate lists of coral scoring poses for the other alliance.
				CORAL_SCORING_POSES_BLUE_LEFT.forEach((pose) -> {
					CORAL_SCORING_POSES_RED_LEFT.add(PoseUtil.flipPose(pose));
				});
				CORAL_SCORING_POSES_BLUE_RIGHT.forEach((pose) -> {
					CORAL_SCORING_POSES_RED_RIGHT.add(PoseUtil.flipPose(pose));
				});

				// Generate a list of algae scoring poses.
				TAG_POSES.forEach((pose) -> {
					Pose2d pose2d = pose.toPose2d();
					ALGAE_SCORING_POSES_BLUE.add(pose2d.transformBy(ALGAE_SCORING_OFFSET));
				});

				// Generate a list of algae scoring poses for the other alliance.
				ALGAE_SCORING_POSES_BLUE.forEach((pose) -> {
					ALGAE_SCORING_POSES_RED.add(PoseUtil.flipPose(pose));
				});
			}
		}

		/**
		 * Constants relating to the processor.
		 */
		public static class Processor {
			/**
			 * AprilTag ID for the processor on the blue alliance.
			 */
			public static final int TAG_ID = 16;
			/**
			 * AprilTag pose for the processor on the blue alliance.
			 */
			public static final Pose3d TAG_POSE;
			/**
			 * Offset from the AprilTag from which scoring should happen.
			 */
			public static final Transform2d SCORING_OFFSET = new Transform2d(Meters.of(0.33 / 2), Meters.of(0), Rotation2d.k180deg);
			/**
			 * Pose from which the robot can score algae on the blue alliance.
			 */
			public static final Pose2d ALGAE_SCORING_POSE_BLUE;
			/**
			 * Pose from which the robot can score algae on the red alliance.
			 */
			public static final Pose2d ALGAE_SCORING_POSE_RED;

			static {
				// Find the tag pose.
				Optional<Pose3d> tagPose = FIELD_LAYOUT.getTagPose(TAG_ID);

				if (tagPose.isPresent()) {
					TAG_POSE = tagPose.get();
				} else {
					TAG_POSE = new Pose3d();
				}

				// Generate scoring poses.
				Pose2d pose2d = TAG_POSE.toPose2d();

				ALGAE_SCORING_POSE_BLUE = pose2d.transformBy(SCORING_OFFSET);
				ALGAE_SCORING_POSE_RED = PoseUtil.flipPose(ALGAE_SCORING_POSE_BLUE);
			}
		}
	}
}
