package io.github.roboblazers7617.limelight;

import com.fasterxml.jackson.annotation.JsonFormat;
import com.fasterxml.jackson.annotation.JsonFormat.Shape;
import com.fasterxml.jackson.annotation.JsonProperty;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import io.github.roboblazers7617.limelight.targets.BarcodeTarget;
import io.github.roboblazers7617.limelight.targets.FiducialTarget;
import io.github.roboblazers7617.limelight.targets.RetroreflectiveTarget;
import io.github.roboblazers7617.limelight.targets.neural.ClassifierTarget;
import io.github.roboblazers7617.limelight.targets.neural.DetectorTarget;

/**
 * Limelight PipelineResult object, parsed from a Limelight's JSON results output.
 */
public class PipelineResult {
	public String error;

	@JsonProperty("pID")
	public double pipelineID;

	@JsonProperty("tl")
	public double latency_pipeline;

	@JsonProperty("cl")
	public double latency_capture;

	public double latency_jsonParse;

	@JsonProperty("ts")
	public double timestamp_LIMELIGHT_publish;

	@JsonProperty("ts_rio")
	public double timestamp_RIOFPGA_capture;

	@JsonProperty("v")
	@JsonFormat(shape = Shape.NUMBER)
	public boolean valid;

	@JsonProperty("botpose")
	public double[] botpose;

	@JsonProperty("botpose_wpired")
	public double[] botpose_wpired;

	@JsonProperty("botpose_wpiblue")
	public double[] botpose_wpiblue;

	@JsonProperty("botpose_tagcount")
	public double botpose_tagcount;

	@JsonProperty("botpose_span")
	public double botpose_span;

	@JsonProperty("botpose_avgdist")
	public double botpose_avgdist;

	@JsonProperty("botpose_avgarea")
	public double botpose_avgarea;

	@JsonProperty("t6c_rs")
	public double[] camerapose_robotspace;

	public Pose3d getBotPose3d() {
		return JsonUtilities.toPose3D(botpose);
	}

	public Pose3d getBotPose3d_wpiRed() {
		return JsonUtilities.toPose3D(botpose_wpired);
	}

	public Pose3d getBotPose3d_wpiBlue() {
		return JsonUtilities.toPose3D(botpose_wpiblue);
	}

	public Pose2d getBotPose2d() {
		return JsonUtilities.toPose2D(botpose);
	}

	public Pose2d getBotPose2d_wpiRed() {
		return JsonUtilities.toPose2D(botpose_wpired);
	}

	public Pose2d getBotPose2d_wpiBlue() {
		return JsonUtilities.toPose2D(botpose_wpiblue);
	}

	@JsonProperty("Retro")
	public RetroreflectiveTarget[] targets_Retro;

	@JsonProperty("Fiducial")
	public FiducialTarget[] targets_Fiducials;

	@JsonProperty("Classifier")
	public io.github.roboblazers7617.limelight.targets.neural.ClassifierTarget[] targets_Classifier;

	@JsonProperty("Detector")
	public io.github.roboblazers7617.limelight.targets.neural.DetectorTarget[] targets_Detector;

	@JsonProperty("Barcode")
	public BarcodeTarget[] targets_Barcode;

	public PipelineResult() {
		botpose = new double[6];
		botpose_wpired = new double[6];
		botpose_wpiblue = new double[6];
		camerapose_robotspace = new double[6];
		targets_Retro = new RetroreflectiveTarget[0];
		targets_Fiducials = new FiducialTarget[0];
		targets_Classifier = new ClassifierTarget[0];
		targets_Detector = new DetectorTarget[0];
		targets_Barcode = new BarcodeTarget[0];
	}
}
