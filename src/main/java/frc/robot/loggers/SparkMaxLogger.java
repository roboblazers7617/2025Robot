package frc.robot.loggers;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(SparkMax.class)
public class SparkMaxLogger extends ClassSpecificLogger<SparkMax> {
	public SparkMaxLogger() {
		super(SparkMax.class);
	}
	
	@Override
	protected void update(EpilogueBackend backend, SparkMax motor) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("Voltage", motor.getBusVoltage());
			backend.log("Percentage", motor.get());
			try {
				// if it doesn't have an encoder it will throw an error
				backend.log("Relative encoder position", motor.getAlternateEncoder().getPosition());
				backend.log("Relative encoder velocity", motor.getAlternateEncoder().getVelocity());
			} catch (Exception e) {}
			try {
				// if it doesn't have an encoder it will throw an error
				backend.log("Absolute encoder position", motor.getAbsoluteEncoder().getPosition());
				backend.log("Absolute encoder velocity", motor.getAbsoluteEncoder().getVelocity());
			} catch (Exception e) {}
		}
		if (Epilogue.shouldLog(Logged.Importance.INFO)) {
			backend.log("Has fault", motor.hasActiveFault());
			backend.log("Has warning", motor.hasActiveWarning());
		}
	}
}