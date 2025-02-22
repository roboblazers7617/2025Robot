package io.github.roboblazers7617.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import io.github.roboblazers7617.limelight.targets.RawFiducialTarget;

/**
 * Represents a 3D Pose Estimate.
 */
public class PoseEstimate {
	public Pose3d pose;
	public double timestampSeconds;
	public double latency;
	public int tagCount;
	public double tagSpan;
	public double avgTagDist;
	public double avgTagArea;

	public RawFiducialTarget[] rawFiducials;
	public boolean isMegaTag2;

	/**
	 * Instantiates a PoseEstimate object with default values
	 */
	public PoseEstimate() {
		this.pose = new Pose3d();
		this.timestampSeconds = 0;
		this.latency = 0;
		this.tagCount = 0;
		this.tagSpan = 0;
		this.avgTagDist = 0;
		this.avgTagArea = 0;
		this.rawFiducials = new RawFiducialTarget[] {};
		this.isMegaTag2 = false;
	}

	public PoseEstimate(Pose3d pose, double timestampSeconds, double latency, int tagCount, double tagSpan, double avgTagDist, double avgTagArea, RawFiducialTarget[] rawFiducials, boolean isMegaTag2) {
		this.pose = pose;
		this.timestampSeconds = timestampSeconds;
		this.latency = latency;
		this.tagCount = tagCount;
		this.tagSpan = tagSpan;
		this.avgTagDist = avgTagDist;
		this.avgTagArea = avgTagArea;
		this.rawFiducials = rawFiducials;
		this.isMegaTag2 = isMegaTag2;
	}

	public Pose3d getPose3d() {
		return pose;
	}

	public Pose2d getPose2d() {
		return pose.toPose2d();
	}

	public RawFiducialTarget[] getDetectedTags() {
		return rawFiducials;
	}

	public double getTimestampSeconds() {
		return timestampSeconds;
	}

	public int getTagCount() {
		return tagCount;
	}
}
