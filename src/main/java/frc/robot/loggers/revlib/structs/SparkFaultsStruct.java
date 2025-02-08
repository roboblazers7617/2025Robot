package frc.robot.loggers.revlib.structs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

/**
 * Struct to represent the data in {@link com.revrobotics.spark.SparkBase.Faults}.
 */
public class SparkFaultsStruct implements Struct<Integer> {
	/**
	 * Static instance of the struct.
	 */
	public static final SparkFaultsStruct inst = new SparkFaultsStruct();

	@Override
	public Class<Integer> getTypeClass() {
		return Integer.class;
	}

	@Override
	public String getTypeName() {
		return "SparkFaults";
	}

	@Override
	public int getSize() {
		return kSizeInt32;
	}

	@Override
	public String getSchema() {
		return ("bool other:1; " + "bool motorType:1; " + "bool sensor:1; " + "bool can:1; " + "bool temperature:1; " + "bool gateDriver:1; " + "bool escEeprom:1; " + "bool firmware:1;");
	}

	@Override
	public Integer unpack(ByteBuffer bb) {
		return bb.getInt();
	}

	@Override
	public void pack(ByteBuffer bb, Integer value) {
		bb.putInt(value.intValue());
	}

	@Override
	public boolean isImmutable() {
		return true;
	}
}
