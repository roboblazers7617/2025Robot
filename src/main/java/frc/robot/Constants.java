// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.io.File;
import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.PoseUtil;
import io.github.roboblazers7617.limelight.LimelightSettings.ImuMode;
import io.github.roboblazers7617.limelight.PoseEstimator.PoseEstimators;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
		public static final double TRANSLATION_SCALE_SLOW = 0.3;
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

	/**
	 * Constants used to configure logging.
	 * <p>
	 * During a competition debug mode should be false to reduce network and CPU usage. All data will still be logged it just won't be accessible until after the match.
	 * <p>
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
		public static final int RIGHT_MOTOR_ID = 22;
		/**
		 * CAN ID for the left elevator motor.
		 */
		public static final int LEFT_MOTOR_ID = 52; // 21

		/**
		 * Elevator kP.
		 */
		public static final double KP = 1.2; // 0.8
		/**
		 * Elevator kI.
		 */
		public static final double KI = 0.0;
		/**
		 * Elevator kD.
		 */
		public static final double KD = 1.0;
		/**
		 * Elevator kS.
		 */
		public static final double KS = 0.35;
		/**
		 * Elevator kG.
		 */
		public static final double KG = 0.25; // 0.14
		/**
		 * Elevator kV.
		 */
		public static final double KV = 6.5;// 6.9

		/**
		 * Elevator kMinOutput as a percentage.
		 */
		public static final double KMIN_OUTPUT = -1.0;
		/**
		 * Elevator kMaxOutput as a percentage.
		 */
		public static final double KMAX_OUTPUT = 1.0;
		/**
		 * Maximum velocity in m/s.
		 */
		// TODO: (Brandon) Update with accurate number. Is the elevator really going to travel 3 feet in one second? Use reca.lc
		public static final double MAX_VELOCITY = 2;
		/**
		 * Maximum acceleration in m/s^2.
		 */
		// TODO: (Brandon) Update with accurate number Use reca.lc
		public static final double MAX_ACCELERATION = 2;

		/**
		 * Maximum position in meters.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_POSITION = 1.44;
		/**
		 * Minimum position in meters.
		 */
		public static final double MIN_POSITION = 0.0;
		/**
		 * This is the maximum position for the elevator to be considered lowered, in meters.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_LOWERED_POSITION = .13;
		/**
		 * Conversion factor from rotation to meters. 3.81cm diameter spool, 16:1 gear ratio
		 */
		public static final double POSITION_CONVERSION_FACTOR = (1 / .2845) / 200;
		/**
		 * Conversion factor from rotation to meters per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;
		/**
		 * Zero offset, meters.
		 */
		public static final double ZERO_OFFSET = 0;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;
		/**
		 * Tolerance for the target to be considered reached in meters.
		 */
		public static final double TOLERANCE = .02;
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
		 * mass: 16lbs
		 */
		/**
		 * CAN Motor ID for the wrist.
		 */
		public static final int MOTOR_ID = 31;

		/**
		 * Wrist kP.
		 */
		public static final double KP = 0.01; // 0.006
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
		public static final double KS = 0.1;// 0.1
		/**
		 * Wrist kG.
		 */
		public static final double KG = 0.3;
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
		public static final double MAX_VELOCITY = 200.0;
		/**
		 * Maximum acceleration in degrees/s^2.
		 */
		// TODO: (Brandon) Update with accurate number
		public static final double MAX_ACCELERATION = 200.0;

		/**
		 * Maximum position in degrees.
		 */
		public static final double MAX_POSITION = 145.0;
		/**
		 * Minimum position when the elevator is not lowered, (determined by MAX_LOWERED_POSITION) in degrees.
		 */

		public static final double MIN_POSITION = -50.0;
		/**
		 * Minimum safe position while the elevator is lowered, in degrees.
		 */
		public static final double SAFE_MIN_POSITION = -40.0;

		/**
		 * Maximum safe position while the elevator is raised (so it doesn't collide with the metal thing on top), in degrees.
		 */
		public static final double SAFE_MAX_POSITION = 130.0;

		/**
		 * Maximum position that the wrist can be while holding an algae (to make sure it doesn't hit the elevator), in degrees.
		 */
		public static final double MAX_ALGAE_POSITION_WITH_ELEVATOR = 45.0;
		/**
		 * Maximum position that the wrist can be while holding an algae if the elevator is fully extended, in degrees.
		 */
		public static final double MAX_ALGAE_POSITION_WITHOUT_ELEVATOR = 95.0;
		/**
		 * Conversion factor from rotation to degrees.
		 */
		public static final double POSITION_CONVERSION_FACTOR = 1.0 / ((10.0 / 58.0) * (18.0 / 58.0) * (30.0 / 12.0)); // first two conversions are gear boxes, third one is chain
		/**
		 * Conversion factor from rotation to degrees per second.
		 */
		public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60.0;
		/**
		 * Zero offset, in rotations, because reasons.
		 */
		public static final double ZERO_OFFSET = 0.73;
		/**
		 * Current limit in amps.
		 */
		public static final int CURRENT_LIMIT = 40;

		/**
		 * Tolerance for the target to be considered reached in degrees.
		 */
		public static final double TOLERANCE = 5;
	}

	/**
	 * Constants with the arm and elevator positions for various positions.
	 * <p>
	 * Wrist position is in degrees, elevator position is in meters.
	 */
	public enum ArmPosition {
		//
		INTAKE_CORAL_CORAL_STATION(WristConstants.MAX_POSITION, ElevatorConstants.MIN_POSITION),
		//
		INTAKE_ALGAE_LEVEL_2(-23, .44),
		//
		INTAKE_ALGAE_LEVEL_3(-23, .854),
		//
		OUTTAKE_CORAL_LEVEL_1(-45, 1), // dummy value
		//
		OUTTAKE_CORAL_LEVEL_2(125, 0.18),
		//
		OUTTAKE_CORAL_LEVEL_3(125, 0.58),
		//
		OUTTAKE_CORAL_LEVEL_4(87.9, 1.25),
		//
		OUTTAKE_CORAL_LEVEL_4_HIGH(87.9, 1.4),
		//
		OUTTAKE_ALGAE_PROCESSOR(-57, .1),
		//
		OUTTAKE_ALGAE_NET(0, 0),
		/** elevator at bottom, wrist open so we don't crush the algae into the robot */
		STOW_ALGAE(48, 0),
		/** wrist up and elevator down */
		STOW(125, ElevatorConstants.MIN_POSITION),
		//
		CLIMB(-45, 1);

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
	 * Constants used to configure the {@link frc.robot.subsystems.EndEffector.EndEffector End Effector}.
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
		 * Used for making numbers apear correctly in software.
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
		 * Intake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_MAIN_INTAKE_SPEED = 0.5;
		/**
		 * Intake motor speed for coral after hitting main beam break. (to be changed and edited later)
		 */
		public static final double CORAL_SECONDARY_INTAKE_SPEED = 0.05;
		/**
		 * Outtake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_OUTAKE_SPEED = 0.5;
		/**
		 * Intake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_INTAKE_SPEED = -0.5;
		/**
		 * Outtake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_OUTAKE_SPEED = 1.0;
		/**
		 * Speed that the motor will use to keep force on the Algae while it is being held.
		 */
		public static final double ALGAE_HOLD_SPEED = -0.1;
		/**
		 * Time (in seconds) that the motors run after beam break detects no coral after using the outtake command
		 */
		public static final double OUTTAKE_WAIT_TIME = 0.2;
		/**
		 * Time (in seconds) that the motors run after algae outake is called to eject algae.
		 */
		public static final double ALGAE_OUTTAKE_RUN_TIME = 0.3;
		// Beam Break constants
		/**
		 * DIO pin for the main beam break.
		 */
		public static final int MAIN_BEAM_BREAK_DIO = 0;
		/**
		 * DIO pin for the secondary beam break.
		 */
		public static final int SECONDARY_BEAM_BREAK_DIO = 1;
		// Limit Switch constants
		/**
		 * DIO pin for limit switch.
		 */
		public static final int LIMIT_SWITCH_DIO = 4;
	}

	/**
	 * Constants used to configure the {@link frc.robot.subsystems.IntakeRamp.Ramp Ramp}.
	 */
	public static class RampConstants {
		/**
		 * Ramp gear ratio.
		 */
		// TODO: set proper gear ratio
		public static final double RAMP_MOTOR_GEAR_RATIO = (1.0 / 60.0);
		/**
		 * Ramp CAN ID.
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
		 * Ramp motor's current limit.
		 */
		public static final int RAMP_MOTOR_CURRENT_LIMIT = 20;
		/*
		 * I Belive we used this to make the numbers apear correctly in the dirvers station?
		 * May be a usesless holdover from arm code test though.
		 */
		public static final double POSITION_CONVERSION_FACTOR = RAMP_MOTOR_GEAR_RATIO * 360.0;
		/**
		 * Ramp Stow Position.
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
			public static final Transform2d CORAL_SCORING_OFFSET = new Transform2d(Meters.of(0.5), Meters.of(0.33 / 2), Rotation2d.k180deg);
			/**
			 * Offset from the AprilTag from which coral scoring into L4 should happen.
			 */
			public static final Transform2d CORAL_SCORING_OFFSET_L4 = new Transform2d(Meters.of(2.5), Meters.of(0.33 / 2), Rotation2d.k180deg);
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
			 * Poses from which the robot can score coral in L4 on the left side on the blue alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_L4_BLUE_LEFT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral in L4 on the right side on the blue alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_L4_BLUE_RIGHT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral in L4 on the left side on the red alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_L4_RED_LEFT = new ArrayList<Pose2d>();
			/**
			 * Poses from which the robot can score coral in L4 on the right side on the red alliance.
			 */
			public static final List<Pose2d> CORAL_SCORING_POSES_L4_RED_RIGHT = new ArrayList<Pose2d>();
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
					CORAL_SCORING_POSES_BLUE_LEFT.add(pose2d.transformBy(PoseUtil.flipTransformY(CORAL_SCORING_OFFSET)));

					// Regular side (L4)
					CORAL_SCORING_POSES_L4_BLUE_RIGHT.add(pose2d.transformBy(CORAL_SCORING_OFFSET_L4));
					// Flipped side (L4)
					CORAL_SCORING_POSES_L4_BLUE_LEFT.add(pose2d.transformBy(PoseUtil.flipTransformY(CORAL_SCORING_OFFSET_L4)));
				});

				// Generate lists of coral scoring poses for the other alliance.
				CORAL_SCORING_POSES_BLUE_LEFT.forEach((pose) -> {
					CORAL_SCORING_POSES_RED_LEFT.add(PoseUtil.flipPoseAlliance(pose));
				});
				CORAL_SCORING_POSES_BLUE_RIGHT.forEach((pose) -> {
					CORAL_SCORING_POSES_RED_RIGHT.add(PoseUtil.flipPoseAlliance(pose));
				});

				// Generate lists of coral scoring poses for L4 for the other alliance.
				CORAL_SCORING_POSES_L4_BLUE_LEFT.forEach((pose) -> {
					CORAL_SCORING_POSES_L4_RED_LEFT.add(PoseUtil.flipPoseAlliance(pose));
				});
				CORAL_SCORING_POSES_L4_BLUE_RIGHT.forEach((pose) -> {
					CORAL_SCORING_POSES_L4_RED_RIGHT.add(PoseUtil.flipPoseAlliance(pose));
				});

				// Generate a list of algae scoring poses.
				TAG_POSES.forEach((pose) -> {
					Pose2d pose2d = pose.toPose2d();
					ALGAE_SCORING_POSES_BLUE.add(pose2d.transformBy(ALGAE_SCORING_OFFSET));
				});

				// Generate a list of algae scoring poses for the other alliance.
				ALGAE_SCORING_POSES_BLUE.forEach((pose) -> {
					ALGAE_SCORING_POSES_RED.add(PoseUtil.flipPoseAlliance(pose));
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
				ALGAE_SCORING_POSE_RED = PoseUtil.flipPoseAlliance(ALGAE_SCORING_POSE_BLUE);
			}
		}
	}

	/**
	 * Poses that the robot can score from.
	 */
	public static class ScoringPoses {
		// TODO: Add poses for pathfinding to the coral station and the cages
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
		 * Poses from which the robot can score coral in L4 on the left side on the blue alliance.
		 */
		public static final List<Pose2d> CORAL_SCORING_POSES_L4_BLUE_LEFT = new ArrayList<Pose2d>();
		/**
		 * Poses from which the robot can score coral in L4 on the right side on the blue alliance.
		 */
		public static final List<Pose2d> CORAL_SCORING_POSES_L4_BLUE_RIGHT = new ArrayList<Pose2d>();
		/**
		 * Poses from which the robot can score coral in L4 on the left side on the red alliance.
		 */
		public static final List<Pose2d> CORAL_SCORING_POSES_L4_RED_LEFT = new ArrayList<Pose2d>();
		/**
		 * Poses from which the robot can score coral in L4 on the right side on the red alliance.
		 */
		public static final List<Pose2d> CORAL_SCORING_POSES_L4_RED_RIGHT = new ArrayList<Pose2d>();
		/**
		 * Poses from which the robot can score algae on the blue alliance.
		 */
		public static final List<Pose2d> ALGAE_SCORING_POSES_BLUE = new ArrayList<Pose2d>();
		/**
		 * Poses from which the robot can score algae on the red alliance.
		 */
		public static final List<Pose2d> ALGAE_SCORING_POSES_RED = new ArrayList<Pose2d>();

		static {
			// Compile CORAL_SCORING_POSES_BLUE_LEFT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_BLUE_LEFT.forEach((pose) -> {
				CORAL_SCORING_POSES_BLUE_LEFT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_BLUE_RIGHT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_BLUE_RIGHT.forEach((pose) -> {
				CORAL_SCORING_POSES_BLUE_RIGHT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_RED_LEFT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_RED_LEFT.forEach((pose) -> {
				CORAL_SCORING_POSES_RED_LEFT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_RED_RIGHT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_RED_RIGHT.forEach((pose) -> {
				CORAL_SCORING_POSES_RED_RIGHT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_L4_BLUE_LEFT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_L4_BLUE_LEFT.forEach((pose) -> {
				CORAL_SCORING_POSES_L4_BLUE_LEFT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_L4_BLUE_RIGHT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_L4_BLUE_RIGHT.forEach((pose) -> {
				CORAL_SCORING_POSES_L4_BLUE_RIGHT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_L4_RED_LEFT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_L4_RED_LEFT.forEach((pose) -> {
				CORAL_SCORING_POSES_L4_RED_LEFT.add(pose);
			});

			// Compile CORAL_SCORING_POSES_L4_RED_RIGHT poses
			FieldConstants.Reef.CORAL_SCORING_POSES_L4_RED_RIGHT.forEach((pose) -> {
				CORAL_SCORING_POSES_L4_RED_RIGHT.add(pose);
			});

			// Compile ALGAE_SCORING_POSES_BLUE poses
			FieldConstants.Reef.ALGAE_SCORING_POSES_BLUE.forEach((pose) -> {
				ALGAE_SCORING_POSES_BLUE.add(pose);
			});
			ALGAE_SCORING_POSES_BLUE.add(FieldConstants.Processor.ALGAE_SCORING_POSE_BLUE);

			// Compile ALGAE_SCORING_POSES_RED poses
			FieldConstants.Reef.ALGAE_SCORING_POSES_RED.forEach((pose) -> {
				ALGAE_SCORING_POSES_RED.add(pose);
			});
			ALGAE_SCORING_POSES_RED.add(FieldConstants.Processor.ALGAE_SCORING_POSE_RED);
		}
	}
}
