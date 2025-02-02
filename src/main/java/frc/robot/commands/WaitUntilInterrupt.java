package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalSource;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BiConsumer;

/**
 * Command that creates an {@link edu.wpi.first.wpilibj.AsynchronousInterrupt} and uses it to trigger a callback, waiting until the callback happens before finishing.
 */
public class WaitUntilInterrupt extends Command {
	/**
	 * Set to true after the callback is called.
	 */
	private AtomicBoolean hasFinished = new AtomicBoolean(false);
	private final AsynchronousInterrupt interrupt;

	/**
	 * Creates a new WaitUntilInterrupt command that finishes when the interrupt is triggered.
	 * <p>
	 * First bool in callback indicates trigger on rising edge, second bool indicates falling edge.
	 *
	 * @param source
	 *            Digital source to await an interrupt on.
	 * @param callback
	 *            Callback to call on interrupt.
	 * @param risingEdge
	 *            Trigger on the rising edge.
	 * @param fallingEdge
	 *            Trigger on the falling edge.
	 */
	public WaitUntilInterrupt(DigitalSource source, BiConsumer<Boolean, Boolean> callback, boolean risingEdge, boolean fallingEdge) {
		interrupt = new AsynchronousInterrupt(source, (rising, falling) -> {
			callback.accept(rising, falling);
			hasFinished.set(true);
		});
		interrupt.setInterruptEdges(risingEdge, fallingEdge);
	}

	/**
	 * Creates a new WaitUntilInterrupt command that finishes when the interrupt is triggered. Triggers on both the rising and falling edges.
	 * <p>
	 * First bool in callback indicates trigger on rising edge, second bool indicates falling edge.
	 *
	 * @param source
	 *            Digital source to await an interrupt on.
	 * @param callback
	 *            Callback to call on interrupt.
	 */
	public WaitUntilInterrupt(DigitalSource source, BiConsumer<Boolean, Boolean> callback) {
		this(source, callback, true, true);
	}

	@Override
	public void initialize() {
		hasFinished.set(false);
		interrupt.enable();
	}

	@Override
	public void end(boolean interrupted) {
		interrupt.disable();
	}

	@Override
	public boolean isFinished() {
		return hasFinished.get();
	}
}
