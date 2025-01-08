// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.util.Units;

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
		 * Mass of the robot.
		 */
		public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
	}
	
	/**
	 * Constants used by the {@link frc.robot.subsystems.Drivetrain}.
	 */
	public static class DrivetrainConstants {
		/**
		 * Maximum speed of the robot in meters per second.
		 */
		public static final double MAX_SPEED = Units.feetToMeters(14.5);
		/**
		 * Directory that contains the YAGSL configuration.
		 */
		public static final File CONFIG_DIR = new File(Filesystem.getDeployDirectory(), "swerve");
	}
	
	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}
}
