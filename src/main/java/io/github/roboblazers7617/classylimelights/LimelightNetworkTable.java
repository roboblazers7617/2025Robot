package io.github.roboblazers7617.classylimelights;

import java.net.MalformedURLException;
import java.net.URL;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightNetworkTable {
	private Limelight limelight;

	public LimelightNetworkTable(Limelight limelight) {
		this.limelight = limelight;
	}

	public NetworkTable getLimelightTable() {
		return NetworkTableInstance.getDefault().getTable(limelight.name);
	}

	public void Flush() {
		NetworkTableInstance.getDefault().flush();
	}

	public NetworkTableEntry getEntry(String entryName) {
		return getLimelightTable().getEntry(entryName);
	}

	public double getDouble(String entryName) {
		return getEntry(entryName).getDouble(0.0);
	}

	public void setDouble(String entryName, double val) {
		getEntry(entryName).setDouble(val);
	}

	public void setDoubleArray(String entryName, double[] val) {
		getEntry(entryName).setDoubleArray(val);
	}

	public double[] getDoubleArray(String entryName) {
		return getEntry(entryName).getDoubleArray(new double[0]);
	}

	public String getString(String entryName) {
		return getEntry(entryName).getString("");
	}

	public String[] getStringArray(String entryName) {
		return getEntry(entryName).getStringArray(new String[0]);
	}

	public URL getLimelightURLString(String request) {
		String urlString = "http://" + limelight.name + ".local:5807/" + request;
		URL url;
		try {
			url = new URL(urlString);
			return url;
		} catch (MalformedURLException e) {
			System.err.println("bad LL URL");
		}
		return null;
	}
}
