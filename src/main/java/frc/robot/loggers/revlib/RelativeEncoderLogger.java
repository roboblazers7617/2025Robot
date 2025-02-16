package frc.robot.loggers.revlib;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom logger for RelativeEncoders.
 */
@CustomLoggerFor(RelativeEncoder.class)
public class RelativeEncoderLogger extends ClassSpecificLogger<RelativeEncoder> {
	/**
	 * Creates a new RelativeEncoderLogger.
	 */
	public RelativeEncoderLogger() {
		super(RelativeEncoder.class);
	}

	@Override
	protected void update(EpilogueBackend backend, RelativeEncoder encoder) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("Position", encoder.getPosition());
			backend.log("Velocity", encoder.getVelocity());
		}
	}
}
