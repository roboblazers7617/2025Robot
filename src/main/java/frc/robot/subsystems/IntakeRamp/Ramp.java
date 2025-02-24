package frc.robot.subsystems.IntakeRamp;

import frc.robot.Constants.RampConstants;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
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
	private final SparkMax rampMotor = new SparkMax(RampConstants.RAMP_MOTOR_CAN_ID, MotorType.kBrushless);
	private final SparkClosedLoopController rampController = rampMotor.getClosedLoopController();

	public Ramp() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(RampConstants.RAMP_MOTOR_CURRENT_LIMIT)
				.apply(RampConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(RampConstants.POSITION_CONVERSION_FACTOR);
		rampMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	private void MoveToPosition(Double position) {
		rampController.setReference(position, ControlType.kPosition);
	}

	/**
	 * Starts the Ramp motor.
	 *
	 * @param position
	 *            set position [0,360]
	 * @return
	 *         Command to run.
	 */
	public Command RampRetract() {
		return this.runOnce(() -> {
			MoveToPosition(RampConstants.RAMP_STOW_POSITION);
		});
	}
}
