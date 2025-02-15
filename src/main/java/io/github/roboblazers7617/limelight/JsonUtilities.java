package io.github.roboblazers7617.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class JsonUtilities {
	static final String sanitizeName(String name) {
		if (name == "" || name == null) {
			return "limelight";
		}
		return name;
	}

	/**
	 * Converts a {@link Translation3d} object to an array of doubles in format [x, y, z]. Measurements are in Meters.
	 *
	 * @param translation
	 *            {@link Translation3d} to convert.
	 * @return Double array containing [x, y, z]
	 */
	public static double[] translation3dToArray(Translation3d translation) {
		return new double[] { translation.getX(), translation.getY(), translation.getZ() };
	}

	/**
	 * Takes a 6-length array of pose data and converts it to a Pose3d object.
	 * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
	 *
	 * @param inData
	 *            Array containing pose data [x, y, z, roll, pitch, yaw]
	 * @return Pose3d object representing the pose, or empty Pose3d if invalid data
	 */
	public static Pose3d toPose3D(double[] inData) {
		if (inData.length < 6) {
			// System.err.println("Bad LL 3D Pose Data!");
			return new Pose3d();
		}
		return new Pose3d(new Translation3d(inData[0], inData[1], inData[2]), new Rotation3d(Units.degreesToRadians(inData[3]), Units.degreesToRadians(inData[4]), Units.degreesToRadians(inData[5])));
	}

	/**
	 * Takes a 6-length array of pose data and converts it to a Pose2d object.
	 * Uses only x, y, and yaw components, ignoring z, roll, and pitch.
	 * Array format: [x, y, z, roll, pitch, yaw] where angles are in degrees.
	 *
	 * @param inData
	 *            Array containing pose data [x, y, z, roll, pitch, yaw]
	 * @return Pose2d object representing the pose, or empty Pose2d if invalid data
	 */
	public static Pose2d toPose2D(double[] inData) {
		if (inData.length < 6) {
			// System.err.println("Bad LL 2D Pose Data!");
			return new Pose2d();
		}
		Translation2d tran2d = new Translation2d(inData[0], inData[1]);
		Rotation2d r2d = new Rotation2d(Units.degreesToRadians(inData[5]));
		return new Pose2d(tran2d, r2d);
	}

	/**
	 * Converts a Pose3d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
	 * Translation components are in meters, rotation components are in degrees.
	 *
	 * @param pose
	 *            The Pose3d object to convert
	 * @return A 6-element array containing [x, y, z, roll, pitch, yaw]
	 */
	public static double[] pose3dToArray(Pose3d pose) {
		double[] result = new double[6];
		result[0] = pose.getTranslation().getX();
		result[1] = pose.getTranslation().getY();
		result[2] = pose.getTranslation().getZ();
		result[3] = Units.radiansToDegrees(pose.getRotation().getX());
		result[4] = Units.radiansToDegrees(pose.getRotation().getY());
		result[5] = Units.radiansToDegrees(pose.getRotation().getZ());
		return result;
	}

	/**
	 * Converts a Pose2d object to an array of doubles in the format [x, y, z, roll, pitch, yaw].
	 * Translation components are in meters, rotation components are in degrees.
	 * Note: z, roll, and pitch will be 0 since Pose2d only contains x, y, and yaw.
	 *
	 * @param pose
	 *            The Pose2d object to convert
	 * @return A 6-element array containing [x, y, 0, 0, 0, yaw]
	 */
	public static double[] pose2dToArray(Pose2d pose) {
		double[] result = new double[6];
		result[0] = pose.getTranslation().getX();
		result[1] = pose.getTranslation().getY();
		result[2] = 0;
		result[3] = Units.radiansToDegrees(0);
		result[4] = Units.radiansToDegrees(0);
		result[5] = Units.radiansToDegrees(pose.getRotation().getRadians());
		return result;
	}

	public static double extractArrayEntry(double[] inData, int position) {
		if (inData.length < position + 1) {
			return 0;
		}
		return inData[position];
	}
}
