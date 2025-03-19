package io.github.roboblazers7617.robot2025.buttonbox;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.util.CombinedRuntimeLoader;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.math.jni.WPIMathJNI;
import edu.wpi.first.util.WPIUtilJNI;

import javax.sound.midi.MidiUnavailableException;

import io.github.roboblazers7617.buttonbox.ButtonBoxClient;
import io.github.roboblazers7617.buttonbox.midi.MIDIUtil;
import io.github.roboblazers7617.buttonbox.midi.controls.PhysicalLEDRGBLaunchpad;
import io.github.roboblazers7617.buttonbox.midi.MIDIDevice;

/**
 * Bridge program to connect the 2025Robot ButtonBox hardware to NetworkTables.
 */
public class ButtonBoxBridge {
	public static void main(String[] args) throws IOException, MidiUnavailableException {
		NetworkTablesJNI.Helper.setExtractOnStaticLoad(false);
		WPIUtilJNI.Helper.setExtractOnStaticLoad(false);
		WPIMathJNI.Helper.setExtractOnStaticLoad(false);

		CombinedRuntimeLoader.loadLibraries(ButtonBoxBridge.class, "wpiutiljni", "wpimathjni", "ntcorejni");

		new ButtonBoxBridge().run();
	}

	public void run() throws MidiUnavailableException {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		inst.startClient4("ButtonBox Bridge");
		inst.setServer("localhost"); // where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
		inst.startDSClient(); // recommended if running on DS computer; this gets the robot IP from the DS

		Optional<MIDIDevice> midiDevice = MIDIUtil.getDeviceByName("CoreMIDI4J - Pico");

		if (midiDevice.isEmpty()) {
			throw new MidiUnavailableException("No MIDI device found.");
		}

		ButtonBoxClient client = new ButtonBoxClient(inst);

		configureControls(client, midiDevice.get());

		while (true) {
			try {
				Thread.sleep(10);
			} catch (InterruptedException ex) {
				System.out.println("interrupted");
				return;
			}
			client.periodic();
		}
	}

	/**
	 * Configures the ButtonBox controls.
	 */
	private void configureControls(ButtonBoxClient client, MIDIDevice midiDevice) {
		// Driver controls
		client.addControl(new PhysicalLEDRGBLaunchpad("Mode LED", (byte) 0, midiDevice));
	}
}
