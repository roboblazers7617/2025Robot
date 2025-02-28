package io.github.roboblazers7617.limelight;

import java.util.Arrays;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import io.github.roboblazers7617.limelight.targets.RawFiducialTarget;
import limelight.networktables.LimelightPoseEstimator.BotPose;

public class PoseEstimator {
	private final NetworkTable networkTable;
	private final PoseEstimators poseEstimator;
	private final DoubleArraySubscriber poseSubscriber;

	protected PoseEstimator(Limelight limelight, PoseEstimators poseEstimator) {
		networkTable = limelight.networkTable;
		this.poseEstimator = poseEstimator;
		poseSubscriber = networkTable.getDoubleArrayTopic(poseEstimator.getEntry()).subscribe(new double[0]);
	}

	public PoseEstimate[] getBotPoseEstimates() {
		return Arrays.stream(poseSubscriber.readQueue())
				.map((poseEstimateArray) -> {
					return genPoseEstimateFromNT(poseEstimateArray);
				})
				.toArray(PoseEstimate[]::new);
	}

	private PoseEstimate genPoseEstimateFromNT(TimestampedDoubleArray poseTimestamped) {
		double[] poseArray = poseTimestamped.value;
		long timestamp = poseTimestamped.timestamp;

		if (poseArray.length == 0) {
			// Handle the case where no data is available
			return null; // or some default PoseEstimate
		}

		var pose = JsonUtilities.toPose3D(poseArray);
		double latency = JsonUtilities.extractArrayEntry(poseArray, 6);
		int tagCount = (int) JsonUtilities.extractArrayEntry(poseArray, 7);
		double tagSpan = JsonUtilities.extractArrayEntry(poseArray, 8);
		double tagDist = JsonUtilities.extractArrayEntry(poseArray, 9);
		double tagArea = JsonUtilities.extractArrayEntry(poseArray, 10);

		// Convert server timestamp from microseconds to seconds and adjust for latency
		double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

		RawFiducialTarget[] rawFiducials = new RawFiducialTarget[tagCount];
		int valsPerFiducial = 7;
		int expectedTotalVals = 11 + valsPerFiducial * tagCount;

		if (poseArray.length != expectedTotalVals) {
			// Don't populate fiducials
		} else {
			for (int i = 0; i < tagCount; i++) {
				int baseIndex = 11 + (i * valsPerFiducial);
				int id = (int) poseArray[baseIndex];
				double txnc = poseArray[baseIndex + 1];
				double tync = poseArray[baseIndex + 2];
				double ta = poseArray[baseIndex + 3];
				double distToCamera = poseArray[baseIndex + 4];
				double distToRobot = poseArray[baseIndex + 5];
				double ambiguity = poseArray[baseIndex + 6];
				rawFiducials[i] = new RawFiducialTarget(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
			}
		}

		return new PoseEstimate(pose, adjustedTimestamp, latency, tagCount, tagSpan, tagDist, tagArea, rawFiducials, poseEstimator.isMegaTag2());
	}

	/**
	 * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
	 * name
	 *
	 * @return Array of RawFiducialTarget objects containing detection details
	 */
	public RawFiducialTarget[] getRawFiducialTargets() {
		var entry = networkTable.getEntry("rawfiducials");
		var rawFiducialArray = entry.getDoubleArray(new double[0]);
		int valsPerEntry = 7;
		if (rawFiducialArray.length % valsPerEntry != 0) {
			return new RawFiducialTarget[0];
		}

		int numFiducials = rawFiducialArray.length / valsPerEntry;
		RawFiducialTarget[] rawFiducials = new RawFiducialTarget[numFiducials];

		for (int i = 0; i < numFiducials; i++) {
			int baseIndex = i * valsPerEntry;
			int id = (int) JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex);
			double txnc = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 1);
			double tync = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 2);
			double ta = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 3);
			double distToCamera = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 4);
			double distToRobot = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 5);
			double ambiguity = JsonUtilities.extractArrayEntry(rawFiducialArray, baseIndex + 6);

			rawFiducials[i] = new RawFiducialTarget(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
		}

		return rawFiducials;
	}

	/**
	 * Prints detailed information about a PoseEstimate to standard output.
	 * Includes timestamp, latency, tag count, tag span, average tag distance,
	 * average tag area, and detailed information about each detected fiducial.
	 *
	 * @param pose
	 *            The PoseEstimate object to print. If null, prints "No PoseEstimate available."
	 */
	public void printPoseEstimate(PoseEstimate pose) {
		if (pose == null) {
			System.out.println("No PoseEstimate available.");
			return;
		}

		System.out.printf("Pose Estimate Information:%n");
		System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
		System.out.printf("Latency: %.3f ms%n", pose.latency);
		System.out.printf("Tag Count: %d%n", pose.tagCount);
		System.out.printf("Tag Span: %.2f meters%n", pose.tagSpan);
		System.out.printf("Average Tag Distance: %.2f meters%n", pose.avgTagDist);
		System.out.printf("Average Tag Area: %.2f%% of image%n", pose.avgTagArea);
		System.out.printf("Is MegaTag2: %b%n", pose.isMegaTag2);
		System.out.println();

		if (pose.rawFiducials == null || pose.rawFiducials.length == 0) {
			System.out.println("No RawFiducials data available.");
			return;
		}

		System.out.println("Raw Fiducials Details:");
		for (int i = 0; i < pose.rawFiducials.length; i++) {
			RawFiducialTarget fiducial = pose.rawFiducials[i];
			System.out.printf(" Fiducial #%d:%n", i + 1);
			System.out.printf("  ID: %d%n", fiducial.id);
			System.out.printf("  TXNC: %.2f%n", fiducial.txnc);
			System.out.printf("  TYNC: %.2f%n", fiducial.tync);
			System.out.printf("  TA: %.2f%n", fiducial.ta);
			System.out.printf("  Distance to Camera: %.2f meters%n", fiducial.distToCamera);
			System.out.printf("  Distance to Robot: %.2f meters%n", fiducial.distToRobot);
			System.out.printf("  Ambiguity: %.2f%n", fiducial.ambiguity);
			System.out.println();
		}
	}

	public Boolean validPoseEstimate(PoseEstimate pose) {
		return pose != null && pose.rawFiducials != null && pose.rawFiducials.length != 0;
	}

	/**
	 * PoseEstimators enum for easier decoding.
	 */
	public enum PoseEstimators {
		/**
		 * (Not Recommended) The robot's pose in the WPILib Red Alliance Coordinate System.
		 */
		RED("botpose_wpired", false),
		/**
		 * (Not Recommended) The robot's pose in the WPILib Red Alliance Coordinate System with MegaTag2.
		 */
		RED_MEGATAG2("botpose_orb_wpired", true),
		/**
		 * (Recommended) The robot's 3D pose in the WPILib Blue Alliance Coordinate System.
		 */
		BLUE("botpose_wpiblue", false),
		/**
		 * (Recommended) The robot's 3D pose in the WPILib Blue Alliance Coordinate System with MegaTag2.
		 */
		BLUE_MEGATAG2("botpose_orb_wpiblue", true);

		/**
		 * {@link Limelight} botpose entry name.
		 */
		private final String entry;
		/**
		 * Is megatag2 pose estimator?
		 */
		private final boolean isMegaTag2;

		/**
		 * Create {@link BotPose} enum with given entry names and megatag2 state.
		 *
		 * @param entry
		 *            Bot Pose entry name for {@link Limelight}
		 * @param megatag2
		 *            MegaTag2 reading.
		 */
		PoseEstimators(String entry, boolean isMegaTag2) {
			this.entry = entry;
			this.isMegaTag2 = isMegaTag2;
		}

		public String getEntry() {
			return entry;
		}

		public Boolean isMegaTag2() {
			return isMegaTag2;
		}
	}
}
