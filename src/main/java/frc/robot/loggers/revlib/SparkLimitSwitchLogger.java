package frc.robot.loggers.revlib;

import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom logger for SparkLimitSwitches.
 */
@CustomLoggerFor(SparkLimitSwitch.class)
public class SparkLimitSwitchLogger extends ClassSpecificLogger<SparkLimitSwitch> {
	/**
	 * Creates a new SparkLimitSwitchLogger.
	 */
	public SparkLimitSwitchLogger() {
		super(SparkLimitSwitch.class);
	}

	@Override
	protected void update(EpilogueBackend backend, SparkLimitSwitch limitSwitch) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("Pressed", limitSwitch.isPressed());
		}
	}
}
