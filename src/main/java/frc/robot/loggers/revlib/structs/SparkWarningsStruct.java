package frc.robot.loggers.revlib.structs;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

/**
 * Struct to represent the data in {@link com.revrobotics.spark.SparkBase.Warnings}.
 */
public class SparkWarningsStruct implements Struct<Integer> {
	/**
	 * Static instance of the struct.
	 */
	public static final SparkWarningsStruct inst = new SparkWarningsStruct();

	@Override
	public Class<Integer> getTypeClass() {
		return Integer.class;
	}

	@Override
	public String getTypeName() {
		return "SparkWarnings";
	}

	@Override
	public int getSize() {
		return kSizeInt32;
	}

	@Override
	public String getSchema() {
		return ("bool brownout:1; " + "bool overcurrent:1; " + "bool escEeprom:1; " + "bool extEeprom:1; " + "bool sensor:1; " + "bool stall:1; " + "bool hasReset:1; " + "bool other:1;");
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
