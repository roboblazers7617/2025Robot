// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.Servo;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * Subsystem that controls the climber.
 */
@Logged
public class Climber extends SubsystemBase {
	/**
	 * Main motor used by the climber.
	 */
	private final SparkMax climberMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_CAN_ID, MotorType.kBrushless);
	/**
	 * Encoder for the {@link #climberMotor}.
	 */
	private final RelativeEncoder climberEncoder;
	/**
	 * Servo used to actuate the climber ratchet.
	 */
	private final Servo rachetServo;

	/**
	 * Creates a new Climber.
	 */
	public Climber() {
		climberMotor.configure(new SparkMaxConfig()
				.idleMode(IdleMode.kBrake)
				.inverted(true)
				.smartCurrentLimit(40)
				.apply(new EncoderConfig().positionConversionFactor(ClimberConstants.CLIMBER_GEAR_RATIO)), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		climberEncoder = climberMotor.getEncoder();
		climberEncoder.setPosition(0.0);

		rachetServo = new Servo(ClimberConstants.SERVO_PWM_PORT);
		rachetServo.setPositionConversionFactor(360);
	}

	/**
	 * Engages the ratchet.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command enableRatchetCommand() {
		return rachetServo.moveCommand(ClimberConstants.SERVO_ENABLED_ANGLE);
	}

	/**
	 * Disengages the ratchet.
	 * 
	 * @return
	 *         Command to run.
	 */
	private Command disableRatchetCommand() {
		return rachetServo.moveCommand(ClimberConstants.SERVO_DISABLED_ANGLE);
	}

	/**
	 * Disengages the ratchet and raises the climber.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command RaiseClimber() {
		return disableRatchetCommand()
				.andThen(Commands.runOnce(() -> {
					climberMotor.set(ClimberConstants.RAISE_CLIMBER_SPEED);
				}, this))
				.andThen(Commands.waitUntil(() -> (climberEncoder.getPosition() >= ClimberConstants.CLIMBER_RAISED_POSITION)))
				.finallyDo(() -> {
					climberMotor.set(0);
				});
	}

	/**
	 * Engages the ratchet and lowers the climber.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command LowerClimber() {
		return enableRatchetCommand()
				.andThen(Commands.runOnce(() -> {
					climberMotor.set(ClimberConstants.LOWER_CLIMBER_SPEED);
				}, this))
				.andThen(Commands.waitUntil(() -> (climberEncoder.getPosition() <= ClimberConstants.CLIMBER_LOWERED_POSITION)))
				.finallyDo(() -> {
					climberMotor.set(0);
				});
	}
}
