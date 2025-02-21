// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
/**
 * Creates a new EndEffector.
 * Subsystem for the robot's End Effector functionality
 */
public class EndEffector extends SubsystemBase {
	private double speed;
	private final SparkMax endEffectorMotor = new SparkMax(EndEffectorConstants.CAN_ID_END_EFFECTOR, MotorType.kBrushless);
	/**
	 * Beam break outputs
	 * True = is holding coral
	 * False = is not holding coral
	 */
	// TODO: #134 Rename to follow coding standards / ease reading code
	private final DigitalInput coralBeamBreak = new DigitalInput(EndEffectorConstants.BEAM_BREAK_DIO);

	public EndEffector() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(EndEffectorConstants.MAX_CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				.apply(EndEffectorConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(EndEffectorConstants.POSITION_CONVERSION_FACTOR);

		endEffectorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		speed = endEffectorMotor.getEncoder().getVelocity();
	}

	private void stopMotor() {
		endEffectorMotor.set(0);
	}

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
	public Command StartMotor(Supplier<Double> speed) {
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
	public Command StopMotor() {
		return this.runOnce(() -> {
			stopMotor();
		});
	}

	public Command CoralIntake() {
		return StartMotor(() -> EndEffectorConstants.CORAL_INTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> coralBeamBreak.get()))
				.finallyDo(this::stopMotor);
	}

	public Command CoralOuttake() {
		return StartMotor(() -> EndEffectorConstants.CORAL_OUTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> !coralBeamBreak.get()))
				.andThen(Commands.waitSeconds(EndEffectorConstants.OUTTAKE_WAIT_TIME))
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeIntake() {
		return StartMotor(() -> EndEffectorConstants.ALGAE_INTAKE_SPEED)
				.andThen(Commands.waitSeconds(EndEffectorConstants.MOTOR_CURRENT_CHECK_DELAY))
				// TODO: #135 Check also for speed = 0 (or some small number)
				.andThen((Commands.waitUntil(() -> endEffectorMotor
						.getOutputCurrent() >= EndEffectorConstants.AlGAE_INTAKE_CURRENT_SHUTOFF_THRESHOLD)))
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeOuttake() {
		return StartMotor(() -> EndEffectorConstants.ALGAE_OUTAKE_SPEED)
				.andThen(Commands.waitSeconds(EndEffectorConstants.ALGAE_OUTTAKE_RUN_TIME))
				.finallyDo(this::stopMotor);
	}
}