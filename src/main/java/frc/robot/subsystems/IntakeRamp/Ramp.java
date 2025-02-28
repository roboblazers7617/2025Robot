package frc.robot.subsystems.IntakeRamp;

import frc.robot.Constants.RampConstants;
import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the robot's Ramp functionality.
 */
@Logged
public class Ramp extends SubsystemBase {
	private final SparkMax rampMotor = new SparkMax(RampConstants.RAMP_MOTOR_CAN_ID, MotorType.kBrushless);
	private final SparkClosedLoopController rampController = rampMotor.getClosedLoopController();
	private double encoderVal;
	private final RelativeEncoder rampEncoder = rampMotor.getEncoder();

	/**
	 * Creates a new Ramp.
	 */
	public Ramp() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kCoast)
				.inverted(false)
				.smartCurrentLimit(RampConstants.RAMP_MOTOR_CURRENT_LIMIT)
				.apply(RampConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(RampConstants.POSITION_CONVERSION_FACTOR);
		rampMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		encoderVal = rampEncoder.getPosition();
	}

	private void MoveToPosition(Double position) {
		rampController.setReference(position, ControlType.kPosition);
	}

	public void periodic() {
		encoderVal = rampEncoder.getPosition();
	}

	/**
	 * Starts the Ramp motor.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command RampRetract() {
		return this.runOnce(() -> {
			MoveToPosition(RampConstants.RAMP_STOW_POSITION);
		});
	}
}
