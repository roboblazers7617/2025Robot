package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.State.class)
public class TrapezoidProfileStateLogger extends ClassSpecificLogger<TrapezoidProfile.State> {
	public TrapezoidProfileStateLogger() {
		super(TrapezoidProfile.State.class);
	}

	@Override
	protected void update(EpilogueBackend backend, TrapezoidProfile.State state) {
		backend.log("Position", state.position);
		backend.log("Velocity", state.velocity);
	}
}
