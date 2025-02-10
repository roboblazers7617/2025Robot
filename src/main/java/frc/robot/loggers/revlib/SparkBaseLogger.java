package frc.robot.loggers.revlib;

import com.revrobotics.spark.SparkBase;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import frc.robot.loggers.revlib.structs.SparkFaultsStruct;
import frc.robot.loggers.revlib.structs.SparkWarningsStruct;

/**
 * Custom logger for SparkBase.
 */
@CustomLoggerFor(SparkBase.class)
public class SparkBaseLogger extends ClassSpecificLogger<SparkBase> {
	/**
	 * Creates a new SparkBaseLogger.
	 */
	public SparkBaseLogger() {
		super(SparkBase.class);
	}

	@Override
	protected void update(EpilogueBackend backend, SparkBase motor) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("Voltage", motor.getBusVoltage());
			backend.log("Percentage", motor.get());
			backend.log("Output Current", motor.getOutputCurrent());
			backend.log("Motor Temperature", motor.getMotorTemperature());
			backend.log("Sticky Faults", motor.getStickyFaults().rawBits, SparkFaultsStruct.inst);
			backend.log("Sticky Warnings", motor.getStickyWarnings().rawBits, SparkWarningsStruct.inst);
		}
		if (Epilogue.shouldLog(Logged.Importance.INFO)) {
			backend.log("Active Faults", motor.getFaults().rawBits, SparkFaultsStruct.inst);
			backend.log("Active Warnings", motor.getWarnings().rawBits, SparkWarningsStruct.inst);
		}
	}
}
