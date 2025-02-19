package io.github.roboblazers7617.limelight;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import io.github.roboblazers7617.limelight.targets.RawFiducialTarget;
import io.github.roboblazers7617.limelight.targets.neural.RawDetection;

public class PipelineDataCollator {
	private final Limelight limelight;
	private final NetworkTable networkTable;
	private static final ObjectMapper mapper = new ObjectMapper().configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

	public PipelineDataCollator(Limelight limelight) {
		this.limelight = limelight;
		networkTable = limelight.networkTable;
	}

	/**
	 * Gets the latest raw neural detector results from NetworkTables
	 * name
	 *
	 * @return Array of RawDetection objects containing detection details
	 */
	public RawDetection[] getRawDetections() {
		NetworkTableEntry entry = limelight.networkTable.getEntry("rawdetections");
		double[] rawDetectionArray = entry.getDoubleArray(new double[0]);
		int valsPerEntry = 12;
		if (rawDetectionArray.length % valsPerEntry != 0) {
			return new RawDetection[0];
		}

		int numDetections = rawDetectionArray.length / valsPerEntry;
		RawDetection[] rawDetections = new RawDetection[numDetections];

		for (int i = 0; i < numDetections; i++) {
			int baseIndex = i * valsPerEntry; // Starting index for this detection's data
			int classId = (int) JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex);
			double txnc = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 1);
			double tync = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 2);
			double ta = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 3);
			double corner0_X = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 4);
			double corner0_Y = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 5);
			double corner1_X = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 6);
			double corner1_Y = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 7);
			double corner2_X = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 8);
			double corner2_Y = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 9);
			double corner3_X = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 10);
			double corner3_Y = JsonUtilities.extractArrayEntry(rawDetectionArray, baseIndex + 11);

			rawDetections[i] = new RawDetection(classId, txnc, tync, ta, corner0_X, corner0_Y, corner1_X, corner1_Y, corner2_X, corner2_Y, corner3_X, corner3_Y);
		}

		return rawDetections;
	}

	/**
	 * Gets the latest raw fiducial/AprilTag detection results from NetworkTables.
	 * name
	 *
	 * @return Array of RawFiducialTarget objects containing detection details
	 */
	public RawFiducialTarget[] getRawFiducialTargets() {
		var entry = limelight.networkTable.getEntry("rawfiducials");
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
	 * Gets the latest JSON results output and returns a LimelightResults object.
	 *
	 * @return LimelightResults object containing all current target data
	 */
	public PipelineResult getLatestResults(boolean showParseTime) {
		long start = System.nanoTime();
		PipelineResult results = new PipelineResult();
		try {
			results = mapper.readValue(getJSONDump(), PipelineResult.class);
		} catch (JsonProcessingException e) {
			results.error = "lljson error: " + e.getMessage();
		}

		long end = System.nanoTime();
		double millis = (end - start) * .000001;
		results.latency_jsonParse = millis;
		if (showParseTime) {
			System.out.printf("lljson: %.2f\r\n", millis);
		}

		return results;
	}

	/**
	 * Does the Limelight have a valid target?
	 *
	 * @return True if a valid target is present, false otherwise
	 */
	public boolean getTV() {
		return 1.0 == networkTable.getDoubleTopic("tv").getEntry(0).get();
	}

	/**
	 * Gets the horizontal offset from the crosshair to the target in degrees.
	 *
	 * @return Horizontal offset angle in degrees
	 */
	public double getTX() {
		return networkTable.getDoubleTopic("tx").getEntry(0).get();
	}

	/**
	 * Gets the vertical offset from the crosshair to the target in degrees.
	 *
	 * @return Vertical offset angle in degrees
	 */
	public double getTY() {
		return networkTable.getDoubleTopic("ty").getEntry(0).get();
	}

	/**
	 * Gets the horizontal offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
	 *
	 * @return Horizontal offset angle in degrees
	 */
	public double getTXNC() {
		return networkTable.getDoubleTopic("txnc").getEntry(0).get();
	}

	/**
	 * Gets the vertical offset from the principal pixel/point to the target in degrees. This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
	 *
	 * @return Vertical offset angle in degrees
	 */
	public double getTYNC() {
		return networkTable.getDoubleTopic("tync").getEntry(0).get();
	}

	/**
	 * Gets the target area as a percentage of the image (0-100%).
	 *
	 * @return Target area percentage (0-100)
	 */
	public double getTA() {
		return networkTable.getDoubleTopic("ta").getEntry(0).get();
	}

	/**
	 * T2D is an array that contains several targeting metrcis
	 *
	 * @return Array containing [targetValid, targetCount, targetLatency, captureLatency, tx, ty, txnc, tync, ta, tid, targetClassIndexDetector,
	 *         targetClassIndexClassifier, targetLongSidePixels, targetShortSidePixels, targetHorizontalExtentPixels, targetVerticalExtentPixels, targetSkewDegrees]
	 */
	public double[] getT2DArray() {
		return networkTable.getDoubleArrayTopic("t2d").getEntry(new double[0]).get();
	}

	/**
	 * Gets the number of targets currently detected.
	 *
	 * @return Number of detected targets
	 */
	public int getTargetCount() {
		double[] t2d = getT2DArray();
		if (t2d.length == 17) {
			return (int) t2d[1];
		}
		return 0;
	}

	/**
	 * Gets the classifier class index from the currently running neural classifier pipeline
	 *
	 * @return Class index from classifier pipeline
	 */
	public int getClassifierClassIndex() {
		double[] t2d = getT2DArray();
		if (t2d.length == 17) {
			return (int) t2d[10];
		}
		return 0;
	}

	/**
	 * Gets the detector class index from the primary result of the currently running neural detector pipeline.
	 *
	 * @return Class index from detector pipeline
	 */
	public int getDetectorClassIndex() {
		double[] t2d = getT2DArray();
		if (t2d.length == 17) {
			return (int) t2d[11];
		}
		return 0;
	}

	/**
	 * Gets the current neural classifier result class name.
	 *
	 * @return Class name string from classifier pipeline
	 */
	public String getClassifierClass() {
		return networkTable.getStringTopic("tcclass").getEntry("").get();
	}

	/**
	 * Gets the primary neural detector result class name.
	 *
	 * @return Class name string from detector pipeline
	 */
	public String getDetectorClass() {
		return networkTable.getStringTopic("tdclass").getEntry("").get();
	}

	/**
	 * Gets the pipeline's processing latency contribution.
	 *
	 * @return Pipeline latency in milliseconds
	 */
	public double getLatency_Pipeline() {
		return networkTable.getDoubleTopic("tl").getEntry(0).get();
	}

	/**
	 * Gets the capture latency.
	 *
	 * @return Capture latency in milliseconds
	 */
	public double getLatency_Capture() {
		return networkTable.getDoubleTopic("cl").getEntry(0).get();
	}

	/**
	 * Gets the active pipeline index.
	 *
	 * @return Current pipeline index (0-9)
	 */
	public double getCurrentPipelineIndex() {
		return networkTable.getDoubleTopic("getpipe").getEntry(0).get();
	}

	/**
	 * Gets the current pipeline type.
	 *
	 * @return Pipeline type string (e.g. "retro", "apriltag", etc)
	 */
	public String getCurrentPipelineType() {
		return networkTable.getStringTopic("getpipetype").getEntry("").get();
	}

	/**
	 * Gets the full JSON results dump.
	 *
	 * @return JSON string containing all current results
	 */
	public String getJSONDump() {
		return networkTable.getStringTopic("json").getEntry("").get();
	}

	/**
	 * Gets the robot's 3D pose with respect to the currently tracked target's coordinate system.
	 *
	 * @return Pose3d object representing the robot's position and orientation relative to the target
	 */
	public Pose3d getBotPose3d_TargetSpace() {
		double[] poseArray = networkTable.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]).get();
		return JsonUtilities.toPose3D(poseArray);
	}

	/**
	 * Gets the camera's 3D pose with respect to the currently tracked target's coordinate system.
	 *
	 * @return Pose3d object representing the camera's position and orientation relative to the target
	 */
	public Pose3d getCameraPose3d_TargetSpace() {
		double[] poseArray = networkTable.getDoubleArrayTopic("camerapose_targetspace").getEntry(new double[0]).get();
		return JsonUtilities.toPose3D(poseArray);
	}

	/**
	 * Gets the target's 3D pose with respect to the camera's coordinate system.
	 *
	 * @return Pose3d object representing the target's position and orientation relative to the camera
	 */
	public Pose3d getTargetPose3d_CameraSpace() {
		double[] poseArray = networkTable.getDoubleArrayTopic("targetpose_cameraspace").getEntry(new double[0]).get();
		return JsonUtilities.toPose3D(poseArray);
	}

	/**
	 * Gets the target's 3D pose with respect to the robot's coordinate system.
	 *
	 * @return Pose3d object representing the target's position and orientation relative to the robot
	 */
	public Pose3d getTargetPose3d_RobotSpace() {
		double[] poseArray = networkTable.getDoubleArrayTopic("targetpose_robotspace").getEntry(new double[0]).get();
		;
		return JsonUtilities.toPose3D(poseArray);
	}

	/**
	 * Gets the camera's 3D pose with respect to the robot's coordinate system.
	 *
	 * @return Pose3d object representing the camera's position and orientation relative to the robot
	 */
	public Pose3d getCameraPose3d_RobotSpace() {
		double[] poseArray = networkTable.getDoubleArrayTopic("camerapose_robotspace").getEntry(new double[0]).get();
		;
		return JsonUtilities.toPose3D(poseArray);
	}

	public double[] getTargetColor() {
		return networkTable.getDoubleArrayTopic("tc").getEntry(new double[0]).get();
	}

	public double getFiducialID() {
		return networkTable.getDoubleTopic("tid").getEntry(0).get();
	}

	public String getNeuralClassID() {
		return networkTable.getStringTopic("tclass").getEntry("").get();
	}

	public String[] getRawBarcodeData() {
		return networkTable.getStringArrayTopic("rawbarcodes").getEntry(new String[0]).get();
	}
}
