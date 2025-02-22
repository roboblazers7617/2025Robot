package io.github.roboblazers7617.limelight;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import io.github.roboblazers7617.limelight.PoseEstimator.PoseEstimators;

public class Limelight {
	public final String name;
	public final NetworkTable networkTable;
	public final PipelineDataCollator dataCollator;
	public final LimelightSettings settings;
	private final DoubleArrayEntry robotOrientationEntry;

	public Limelight(String name) {
		this.name = JsonUtilities.sanitizeName(name);

		networkTable = NetworkTableInstance.getDefault().getTable(name);
		settings = new LimelightSettings(this);
		dataCollator = new PipelineDataCollator(this);

		robotOrientationEntry = networkTable.getDoubleArrayTopic("robot_orientation_set").getEntry(new double[0]);
	}

	public void setRobotOrientation(Rotation3d rotation) {
		robotOrientationEntry.set(new double[] { rotation.getMeasureZ().in(Degrees), 0.0, rotation.getMeasureY().in(Degrees), 0, rotation.getMeasureX().in(Degrees), 0 });
		networkTable.getInstance().flush();
	}

	public PoseEstimator makePoseEstimator(PoseEstimators estimator) {
		return new PoseEstimator(this, estimator);
	}

	public URL getLimelightURLString(String request) {
		String urlString = "http://" + name + ".local:5807/" + request;
		URL url;
		try {
			url = new URL(urlString);
			return url;
		} catch (MalformedURLException e) {
			System.err.println("bad LL URL");
		}
		return null;
	}

	/**
	 * Asynchronously take snapshot.
	 */
	public void snapshot(String snapshotName) {
		CompletableFuture.supplyAsync(() -> {
			return SYNCH_TAKESNAPSHOT(snapshotName);
		});
	}

	private boolean SYNCH_TAKESNAPSHOT(String snapshotName) {
		URL url = getLimelightURLString("capturesnapshot");
		try {
			HttpURLConnection connection = (HttpURLConnection) url.openConnection();
			connection.setRequestMethod("GET");
			if (snapshotName != null && snapshotName != "") {
				connection.setRequestProperty("snapname", snapshotName);
			}

			int responseCode = connection.getResponseCode();
			if (responseCode == 200) {
				return true;
			} else {
				System.err.println("Bad LL Request");
			}
		} catch (IOException e) {
			System.err.println(e.getMessage());
		}
		return false;
	}
}
