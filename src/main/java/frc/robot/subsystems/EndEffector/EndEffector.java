// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Subsystem for the robot's End Effector functionality
 */
@Logged
public class EndEffector extends SubsystemBase {
	@Logged
	private final SparkMax endEffectorMotor = new SparkMax(EndEffectorConstants.CAN_ID_END_EFFECTOR, MotorType.kBrushless);
	@Logged
	private final RelativeEncoder endEffectEncoder = endEffectorMotor.getEncoder();

	/**
	 * Beam break outputs
	 * True = is NOT holding coral
	 * False = is holding coral
	 */
	// TODO: #134 Rename to follow coding standards / ease reading code
	@Logged
	private final DigitalInput isNotHoldingCoral = new DigitalInput(EndEffectorConstants.BEAM_BREAK_DIO);

	/**
	 * Creates a new EndEffector.
	 */
	public EndEffector() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(EndEffectorConstants.MAX_CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				.inverted(true)
				.apply(EndEffectorConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(EndEffectorConstants.POSITION_CONVERSION_FACTOR);

		endEffectorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	/**
	 * Sets motor speed to zero.
	 */
	private void stopMotor() {
		endEffectorMotor.set(0);
	}

	/**
	 * Will start the motor with the inputed speed.
	 *
	 * @param speed
	 *            Speed to set [-1,1].
	 */
	private void startMotor(Double speed) {
		endEffectorMotor.set(speed);
	}

	/**
	 * Starts the intake motor.
	 *
	 * @param speed
	 *            Speed to set [-1,1].
	 * @return
	 *         Command to run.
	 */
	public Command StartMotorCommand(Supplier<Double> speed) {
		return this.runOnce(() -> {
			startMotor(speed.get());
		});
	}

	/**
	 * Stops the intake motor.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command StopIntakeMotor() {
		return this.runOnce(() -> {
			stopMotor();
		});
	}

	public Command CoralIntake() {
		return StartMotorCommand(() -> EndEffectorConstants.CORAL_INTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> !isNotHoldingCoral.get()))
				.finallyDo(this::stopMotor);
	}

	public Command CoralOuttake() {
		return StartMotorCommand(() -> EndEffectorConstants.CORAL_OUTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> isNotHoldingCoral.get()))
				.andThen(Commands.waitSeconds(EndEffectorConstants.OUTTAKE_WAIT_TIME))
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeIntake() {
		return StartMotorCommand(() -> EndEffectorConstants.ALGAE_INTAKE_SPEED)
				.andThen(Commands.waitSeconds(EndEffectorConstants.MOTOR_CURRENT_CHECK_DELAY))
				.andThen((Commands.waitUntil(() -> endEffectorMotor
						.getOutputCurrent() >= EndEffectorConstants.AlGAE_INTAKE_CURRENT_SHUTOFF_THRESHOLD && endEffectorMotor.getEncoder().getVelocity() <= EndEffectorConstants.ALGAE_INTAKE_MINIMUM_SHUTOFF_SPEED)))
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeOuttake() {
		return StartMotorCommand(() -> EndEffectorConstants.ALGAE_OUTAKE_SPEED)
				.andThen(Commands.waitSeconds(EndEffectorConstants.ALGAE_OUTTAKE_RUN_TIME))
				.finallyDo(this::stopMotor);
	}

	public boolean isHoldingAlage() {
		return false;
	}

	public boolean isHoldingCoral() {
		return !isNotHoldingCoral.get();
	}
}
