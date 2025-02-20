package frc.robot.subsystems;

import frc.robot.Constants.RampConstants;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
/**
 * Creates a new Ramp.
 * Subsystem for the robot's Ramp functionality
 */
public class Ramp extends SubsystemBase {
	private double speed;
	private final SparkMax rampMotor = new SparkMax(RampConstants.RAMP_MOTOR_CAN_ID, MotorType.kBrushless);

	public Ramp() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(RampConstants.RAMP_MOTOR_CURRENT_LIMIT)
				.apply(RampConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(RampConstants.POSITION_CONVERSION_FACTOR);

		rampMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		speed = rampMotor.getEncoder().getVelocity();
	}

	private void stopMotor() {
		rampMotor.set(0);
	}

	private void startMotor(Double speed) {
		rampMotor.set(speed);
	}

	/**
	 * Starts the Ramp motor.
	 *
	 * @param speed
	 *            Speed to set [-1,1].
	 * @return
	 *         Command to run.
	 */
	public Command StartMotor(Supplier<Double> speed) {
		return this.runOnce(() -> {
			startMotor(speed.get());
		});
	}

	/**
	 * Stops the Ramp motor.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command StopMotor() {
		return this.runOnce(() -> {
			stopMotor();
		});
	}

	// need some way to stop the motor probably not a sensor
	public Command RampDeploy() {
		return StartMotor(() -> RampConstants.RAMP_DEPLOY_SPEED)
				// .andThen(Commands.waitUntil(() -> coralBeamBreak.get()))
				.finallyDo(this::stopMotor);
	}

	public Command RampRetract() {
		return StartMotor(() -> RampConstants.RAMP_RETRACT_SPEED)
				// .andThen(Commands.waitSeconds(EndEffectorConstants.OUTTAKE_WAIT_TIME))
				.finallyDo(this::stopMotor);
	}
}
