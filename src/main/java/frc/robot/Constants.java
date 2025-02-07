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
import java.util.ArrayList;
import java.util.List;
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
		public static final double ROBOT_MASS = Pounds.of(100).in(Kilograms);
		/**
		 * Matter representing the robot chassis.
		 */
		public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Inches.of(8).in(Meters)), ROBOT_MASS);
	}

	/**
	 * Constants used by the {@link frc.robot.subsystems.Drivetrain}.
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
			public static final LinearAcceleration MAX_LINEAR_ACCELERATION = MetersPerSecondPerSecond.of(4.0);
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
	 * Constants used to configure the autonomous program.
	 */
	public static class ClimberConstants {
		/**
		 * Port for the right climber motor.
		 */
		// TODO: make sure these get set
		public static final int RIGHT_CLIMBER_PORT = 0;
		/**
		 * Port for the left climber motor.
		 */
		public static final int LEFT_CLIMBER_PORT = 1;

		/**
		 * Port for the ramp pivot motor.
		 */
		public static final int RAMP_PIVOT_PORT = 2;
	}

	/**
	 * Constants used to configure the elevator.
	 */
	public static class ElevatorConstants {
		// TODO: (Brandon) Update with correct values
		/**
		 * CAN ID for the right elevator motor.
		 */
		public static final int RIGHT_MOTOR_ID = -1;
		/**
		 * CAN ID for the left elevator motor.
		 */
		public static final int LEFT_MOTOR_ID = -1;

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
		 * Elevator kV.
		 */
		public static final double KV = 0;
		/**
		 * Elevator kA.
		 */
		public static final double KA = 0;
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
		 * Conversion factor from rotation to meters.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double POSITION_CONVERSION_FACTOR = 1; // TODO
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
		/**
		 * CAN Motor ID for the wrist.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final int MOTOR_ID = -1;

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
		public static final double KV = 0;

		/**
		 * Wrist kMinOutput.
		 */
		public static final double KMIN_OUTPUT = -.3;
		/**
		 * Wrist kMaxOutput.
		 */
		public static final double KMAX_OUTPUT = .3;
		/**
		 * Maximum velocity in m/s.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_VELOCITY = 1;
		/**
		 * Maximum acceleration in m/s^2.
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
		// TODO: (Brandon) Update with accurate number
		public static final double POSITION_CONVERSION_FACTOR = 1; // TODO
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
			 * Offset from the AprilTag from which scoring should happen.
			 */
			public static final Transform2d SCORING_OFFSET = new Transform2d(Meters.of(0.33 / 2), Meters.of(0.5), new Rotation2d(0));
			/**
			 * Poses from which the robot can score on the blue alliance.
			 */
			public static final List<Pose2d> SCORING_POSES_BLUE = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score on the red alliance.
			 */
			public static final List<Pose2d> SCORING_POSES_RED = new ArrayList<Pose2d>();

			static {
				// Generate a list of tag poses.
				TAG_IDS.forEach((id) -> {
					Optional<Pose3d> tagPose = FIELD_LAYOUT.getTagPose(id);

					if (tagPose.isPresent()) {
						TAG_POSES.add(tagPose.get());
					}
				});

				// Generate a list of scoring poses.
				TAG_POSES.forEach((pose) -> {
					Pose2d pose2d = pose.toPose2d();

					// Regular side
					SCORING_POSES_BLUE.add(pose2d.transformBy(SCORING_OFFSET));
					// Flipped side
					SCORING_POSES_BLUE.add(pose2d.transformBy(new Transform2d(SCORING_OFFSET.getMeasureX(), SCORING_OFFSET.getMeasureY().times(-1), SCORING_OFFSET.getRotation())));
				});

				// Generate a list of scoring poses for the other alliance.
				SCORING_POSES_BLUE.forEach((pose) -> {
					SCORING_POSES_RED.add(PoseUtil.flipPose(pose));
				});
			}
		}
	}
}
