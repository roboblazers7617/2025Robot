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

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.PoseUtil;
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
		// TODO: (Max) Need to update with actual weight of robot
		public static final double ROBOT_MASS = Pounds.of(100).in(Kilograms);
		/**
		 * Matter representing the robot chassis.
		 */
		// TODO: (Max) Need to udpate with actual COG
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
		 * Translation axis scaling. Changes the overall maximum speed of the drivetrain.
		 */
		public static final double TRANSLATION_SCALE = 0.8;
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
		 * Joystick deadband.
		 */
		public static final double DEADBAND = 0.1;
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

	/**
	 * Constants used to configure logging.
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will still be logged it just won't be accessible until after the match.
	 * During testing debug mode should be true to allow for real-time data viewing.
	 */
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
		// public static final List<Double> TAGS_TO_TRACK = IntStream.range(1, 23).asDoubleStream().boxed().toList();
		/**
		 * Use MegaTag2 for pose estimation.
		 */
		public static final boolean ENABLE_MEGATAG2 = true;
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

		/**
		 * Port for the ramp pivot motor.
		 */
		// TODO: (Sam) Please update with correct values
		public static final int RAMP_PIVOT_PORT = 2;
	}

	// wrist is 3:1
	/**
	 * Constants used to configure the elevator.
	 */
	public static class ElevatorConstants {
		/*
		 * A bunch of stuff about the elevator
		 * Neos: 2
		 * ratio: 16:1
		 * current limit: 40 amp
		 * position conversion factor:
		 * mass: Mass - (2 * 6lbs constant force spring)
		 * spool diamaeter: 3.8cm
		 */
		/**
		 * CAN ID for the right elevator motor.
		 */
		public static final int RIGHT_MOTOR_ID = 21;
		/**
		 * CAN ID for the left elevator motor.
		 */
		public static final int LEFT_MOTOR_ID = 22;

		/**
		 * Elevator kP.
		 */
		public static final double KP = 0.1;
		/**
		 * Elevator kI.
		 */
		public static final double KI = 0.0;
		/**
		 * Elevator kD.
		 */
		public static final double KD = 0.0;
		/**
		 * Elevator kS.
		 */
		public static final double KS = 0;
		/**
		 * Elevator kG.
		 */
		public static final double KG = 0;
		/**
		 * Elevator kV.
		 */
		public static final double KV = 0; // Leave as zero Max motion will take care of this

		/**
		 * Elevator kMinOutput as a percentage.
		 */
		public static final double KMIN_OUTPUT = -.3;
		/**
		 * Elevator kMaxOutput as a percentage.
		 */
		public static final double KMAX_OUTPUT = .3;
		/**
		 * Maximum velocity in m/s.
		 */
		// TODO: (Brandon) Update with accurate number. Is the elevator really going to travel 3 feet in one second? Use reca.lc
		public static final double MAX_VELOCITY = 1;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		// TODO: (Brandon) Update with accurate number Use reca.lc
		public static final double MAX_ACCELERATION = 1;

		/**
		 * Maximum position in meters.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_POSITION = 1;
		/**
		 * Minimum position in meters.
		 */
		public static final double MIN_POSITION = 0;
		/**
		 * Minimum safe position while the wrist is lowered, in meters.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double SAFE_MIN_POSITION = 0.2;
		/**
		 * Conversion factor from rotation to meters. 3.81cm diameter spool, 16:1 gear ratio
		 */
		public static final double POSITION_CONVERSION_FACTOR = 3.81 / 100 / 16; // TODO: check
		/**
		 * Conversion factor from rotation to meters per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
		/**
		 * Zero offset, MUST BE [0,1).
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 20;
	}

	/**
	 * Constants used to configure the wrist.
	 */
	public static class WristConstants {
		/*
		 * A bunch of stuff about the wrist
		 * Neos: 1
		 * ratio: 10:58 then 18:58
		 * current limit: 40 amp
		 * center of mass distance:
		 * position conversion factor: 1/(10/58 * 18/58)
		 * mass:
		 */
		/**
		 * CAN Motor ID for the wrist.
		 */
		public static final int MOTOR_ID = 31;

		/**
		 * Wrist kP.
		 */
		public static final double KP = 0.1;
		/**
		 * Wrist kI.
		 */
		public static final double KI = 0.0;
		/**
		 * Wrist kD.
		 */
		public static final double KD = 0.0;

		/**
		 * Wrist kS.
		 */
		public static final double KS = 0;
		/**
		 * Wrist kG.
		 */
		public static final double KG = 0;
		/**
		 * Wrist kV.
		 */
		public static final double KV = 0; // Leave as zero, Max motion will take care of this

		/**
		 * Wrist kMinOutput.
		 */
		public static final double KMIN_OUTPUT = -.3;
		/**
		 * Wrist kMaxOutput.
		 */
		public static final double KMAX_OUTPUT = .3;
		/**
		 * Maximum velocity in degrees/s.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_VELOCITY = 1;
		/**
		 * Maximum acceleration in degrees/s^2.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_ACCELERATION = 1;

		/**
		 * Maximum position in degrees.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_POSITION = 90;
		/**
		 * Minimum position in degrees.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MIN_POSITION = 0;
		/**
		 * Minimum safe position while the elevator is lowered, in degrees.
		 */
		public static final double SAFE_MIN_POSITION = 10;
		/**
		 * Conversion factor from rotation to meters.
		 */
		public static final double POSITION_CONVERSION_FACTOR = 1 / ((10 / 58) * (18 / 58)); // TODO: check
		/**
		 * Conversion factor from rotation to meters per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
		/**
		 * Zero offset, MUST BE [0,1).
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 20;
	}

	public enum ArmPosition {
		//
		INTAKE_CORAL_CORAL_STATION(1.5, 0),
		//
		INTAKE_ALGAE_LEVEL_2(1, 0),
		//
		INTAKE_ALGAE_LEVEL_3(2, 0),
		//
		OUTTAKE_CORAL_LEVEL_1(0, 0),
		//
		OUTTAKE_CORAL_LEVEL_2(0, 0),
		//
		OUTTAKE_CORAL_LEVEL_3(0, 0),
		//
		OUTTAKE_CORAL_LEVEL_4(0, 0),
		//
		OUTTAKE_ALGAE_PROCESSOR(0, 0),
		//
		OUTTAKE_ALGAE_NET(0, 0),
		//
		STOW_ALGAE(0, 0),
		//
		STOW_CORAL(0, 0),
		//
		CLIMB(0, 0);

		/**
		 * The elevator position in meters.
		 */
		public final double ELEVATOR_POSITION;
		/**
		 * The wrist position in degrees.
		 */
		public final double WRIST_POSITION;

		ArmPosition(double WRIST_POSITION, double ELEVATOR_POSITION) {
			this.WRIST_POSITION = WRIST_POSITION;
			this.ELEVATOR_POSITION = ELEVATOR_POSITION;
		}
	}

	/**
	 * Constants that describe the physical layout of the field.
	 */
	public static class FieldConstants {
		/**
		 * AprilTag Field Layout for the current game.
		 */
		public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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
