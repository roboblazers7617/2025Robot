package frc.robot.loggers.revlib;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/**
 * Custom logger for AbsoluteEncoders.
 */
@CustomLoggerFor(AbsoluteEncoder.class)
public class AbsoluteEncoderLogger extends ClassSpecificLogger<AbsoluteEncoder> {
	/**
	 * Creates a new AbsoluteEncoderLogger.
	 */
	public AbsoluteEncoderLogger() {
		super(AbsoluteEncoder.class);
	}

	@Override
	protected void update(EpilogueBackend backend, AbsoluteEncoder encoder) {
		if (Epilogue.shouldLog(Logged.Importance.DEBUG)) {
			backend.log("Position", encoder.getPosition());
			backend.log("Velocity", encoder.getVelocity());
		}
	}
}
