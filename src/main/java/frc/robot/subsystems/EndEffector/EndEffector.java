// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.EndEffectorConstants;

import java.util.concurrent.BlockingDeque;
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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Subsystem for the robot's End Effector functionality
 */
@Logged
public class EndEffector extends SubsystemBase {
	// Private variables
	private double speed;
	@Logged
	private final SparkMax endEffectorMotor = new SparkMax(EndEffectorConstants.CAN_ID_END_EFFECTOR, MotorType.kBrushless);
	@Logged
	private final RelativeEncoder endEffectEncoder = endEffectorMotor.getEncoder();
	@NotLogged
	private final RobotContainer robotContainer;

	// Coral Inputs
	/**
	 * Beam break outputs
	 * True = is NOT holding coral
	 * False = is holding coral
	 */
	private final DigitalInput isNotHoldingCoral = new DigitalInput(EndEffectorConstants.MAIN_BEAM_BREAK_DIO);
	/**
	 * Beam break outputs
	 * True = is NOT holding coral
	 * False = is holding coral
	 */
	private final DigitalInput isNotHoldingCoralAdjuster = new DigitalInput(EndEffectorConstants.SECONDARY_BEAM_BREAK_DIO);
	// Algae Inputs
	private final DigitalInput isHoldingAlgaeInput = new DigitalInput(EndEffectorConstants.LIMIT_SWITCH_DIO);
	private boolean algae = isHoldingAlgaeInput.get();

	/**
	 * Creates a new EndEffector.
	 */
	public EndEffector(RobotContainer robotContainer) {
		this.robotContainer = robotContainer;
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(EndEffectorConstants.MAX_CURRENT_LIMIT)
				.idleMode(IdleMode.kBrake)
				.inverted(true)
				.apply(EndEffectorConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(EndEffectorConstants.POSITION_CONVERSION_FACTOR);
		endEffectorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		speed = endEffectEncoder.getVelocity();
		// if (algae = true && speed <= -0.1 && speed >= -0.2) {
		// StartMotorCommand(() -> EndEffectorConstants.ALGAE_HOLD_SPEED);
		// System.out.println("uh dang");
		// }
	}

	public boolean isHoldingAlgae() {
		return !isHoldingAlgaeInput.get();
	}

	public boolean isHoldingCoral() {
		return !isNotHoldingCoral.get();
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
		return StartMotorCommand(() -> EndEffectorConstants.CORAL_MAIN_INTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> !isNotHoldingCoral.get()))
				.andThen(StartMotorCommand(() -> EndEffectorConstants.CORAL_SECONDARY_INTAKE_SPEED))
				.andThen(Commands.waitUntil(() -> !isNotHoldingCoralAdjuster.get()))
				.finallyDo(this::stopMotor);
	}

	public Command CoralOuttake() {
		return StartMotorCommand(() -> EndEffectorConstants.CORAL_OUTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> isNotHoldingCoralAdjuster.get()))
				.alongWith(Commands.waitUntil(() -> isNotHoldingCoral.get()))
				.finallyDo(this::stopMotor);
	}

	public Command CoralBackup() {
		return StartMotorCommand(() -> EndEffectorConstants.CORAL_BACKUP_SPEED)
				.andThen(Commands.waitUntil(() -> isNotHoldingCoralAdjuster.get()))
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeIntake() {
		return StartMotorCommand(() -> EndEffectorConstants.ALGAE_INTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> !isHoldingAlgaeInput.get()))
				// .andThen(StartMotorCommand(() -> EndEffectorConstants.ALGAE_HOLD_SPEED))
				// .andThen(Commands.waitSeconds(20))// temp to allow time to test
				.finallyDo(this::stopMotor);
	}

	public Command AlgaeOuttake() {
		return StartMotorCommand(() -> EndEffectorConstants.ALGAE_OUTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> isHoldingAlgaeInput.get()))
				.andThen(Commands.waitSeconds(EndEffectorConstants.ALGAE_OUTTAKE_RUN_TIME))
				.finallyDo(this::stopMotor);
	}
}
