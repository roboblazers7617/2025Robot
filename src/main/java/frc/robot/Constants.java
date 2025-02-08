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

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.Filesystem;
import swervelib.math.Matter;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
		 * YAGSL telemetry verbosity.
		 */
		public static final TelemetryVerbosity TELEMETRY_VERBOSITY = TelemetryVerbosity.HIGH;
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
	 * Constants used to configure the end effector.
	 */
	public static class EndEffectorConstants {
		/**
		 * End Effector's motor gear ratio.
		 */
		// TODO:Set correct gear ratio once finalized
		public static final double GEAR_RATIO_END_EFFECTOR_MOTOR = (1.0 / 1.0);
		/**
		 * Neo's current limit
		 */
		public static final int MAX_CURRENT_LIMIT = 40;
		/**
		 * End Effector's Spark Max CAN_ID
		 */
		public static final int CAN_ID_END_EFFECTOR = (41);
		/*
		 * I Belive we used this to make the numbers apear correctly in the dirvers station
		 */
		public static final double POSITION_CONVERSION_FACTOR = GEAR_RATIO_END_EFFECTOR_MOTOR * 360.0;
		/**
		 * TODO: PID values block to be updated with actual values
		 * PID config for the motor controller.
		 */
		public static final ClosedLoopConfig CLOSED_LOOP_CONFIG = new ClosedLoopConfig()
				.p(0)
				.i(0)
				.d(0);
		/**
		 * DIO pin for the beam break.
		 */
		public static final int BEAM_BREAK_DIO = 1;
		/**
		 * Intake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_INTAKE_SPEED = 0.2;
		/**
		 * Outtake motor speed for coral. (to be changed and edited later)
		 */
		public static final double CORAL_OUTAKE_SPEED = -0.2;
		/**
		 * Intake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_INTAKE_SPEED = 0.2;
		/**
		 * Outtake motor speed for algae. (to be changed and edited later)
		 */
		public static final double ALGAE_OUTAKE_SPEED = -0.5;
		/**
		 * Time (in seconds) that the motors run after beam break detects no coral/algae.
		 */
		public static final double WAIT_TIME = 0.3;
		/**
		 * limit to the current before it shuts off the motor for the Algae Intake system.
		 */
		public static final double AlGAE_INTAKE_CURRENT_LIMIT = 20.0;
	}
}
