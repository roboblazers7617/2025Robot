package io.github.roboblazers7617.limelight.targets;

import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import io.github.roboblazers7617.limelight.JsonUtilities;

/**
 * Represents an AprilTag/Fiducial Target Result extracted from JSON Output
 */
public class FiducialTarget {
	@JsonProperty("fID")
	public double fiducialID;

	@JsonProperty("fam")
	public String fiducialFamily;

	@JsonProperty("t6c_ts")
	private double[] cameraPose_TargetSpace;

	@JsonProperty("t6r_fs")
	private double[] robotPose_FieldSpace;

	@JsonProperty("t6r_ts")
	private double[] robotPose_TargetSpace;

	@JsonProperty("t6t_cs")
	private double[] targetPose_CameraSpace;

	@JsonProperty("t6t_rs")
	private double[] targetPose_RobotSpace;

	public Pose3d getCameraPose_TargetSpace() {
		return JsonUtilities.toPose3D(cameraPose_TargetSpace);
	}

	public Pose3d getRobotPose_FieldSpace() {
		return JsonUtilities.toPose3D(robotPose_FieldSpace);
	}

	public Pose3d getRobotPose_TargetSpace() {
		return JsonUtilities.toPose3D(robotPose_TargetSpace);
	}

	public Pose3d getTargetPose_CameraSpace() {
		return JsonUtilities.toPose3D(targetPose_CameraSpace);
	}

	public Pose3d getTargetPose_RobotSpace() {
		return JsonUtilities.toPose3D(targetPose_RobotSpace);
	}

	public Pose2d getCameraPose_TargetSpace2D() {
		return JsonUtilities.toPose2D(cameraPose_TargetSpace);
	}

	public Pose2d getRobotPose_FieldSpace2D() {
		return JsonUtilities.toPose2D(robotPose_FieldSpace);
	}

	public Pose2d getRobotPose_TargetSpace2D() {
		return JsonUtilities.toPose2D(robotPose_TargetSpace);
	}

	public Pose2d getTargetPose_CameraSpace2D() {
		return JsonUtilities.toPose2D(targetPose_CameraSpace);
	}

	public Pose2d getTargetPose_RobotSpace2D() {
		return JsonUtilities.toPose2D(targetPose_RobotSpace);
	}

	@JsonProperty("ta")
	public double ta;

	@JsonProperty("tx")
	public double tx;

	@JsonProperty("ty")
	public double ty;

	@JsonProperty("txp")
	public double tx_pixels;

	@JsonProperty("typ")
	public double ty_pixels;

	@JsonProperty("tx_nocross")
	public double tx_nocrosshair;

	@JsonProperty("ty_nocross")
	public double ty_nocrosshair;

	@JsonProperty("ts")
	public double ts;

	public FiducialTarget() {
		cameraPose_TargetSpace = new double[6];
		robotPose_FieldSpace = new double[6];
		robotPose_TargetSpace = new double[6];
		targetPose_CameraSpace = new double[6];
		targetPose_RobotSpace = new double[6];
	}
}
