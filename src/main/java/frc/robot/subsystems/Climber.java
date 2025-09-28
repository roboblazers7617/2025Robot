// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.WristConstants;

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
	// private final RelativeEncoder climberEncoder;
	private final AbsoluteEncoder climberAbsoluteEncoder;
	private final RelativeEncoder climberRelativeEncoder;

	/**
	 * Servo used to actuate the climber ratchet.
	 */
	private final Servo rachetServo;

	/*
	 * Creates a new Climber.
	 */
	public Climber() {
		SparkBaseConfig climberConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake)
				.inverted(true)
				.smartCurrentLimit(40);
		climberConfig.absoluteEncoder
				.positionConversionFactor(1)
				.zeroOffset(ClimberConstants.ABSOLUTE_ENCODER_OFFSET)
				.inverted(false);
		climberConfig.encoder.positionConversionFactor(ClimberConstants.CLIMBER_GEAR_RATIO);
		climberMotor.configure(climberConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		climberAbsoluteEncoder = climberMotor.getAbsoluteEncoder();

		climberRelativeEncoder = climberMotor.getEncoder();

		rachetServo = new Servo(ClimberConstants.SERVO_PWM_PORT);
	}

	/**
	 * Engages the ratchet.
	 */
	private void enableRatchet() {
		rachetServo.setAngle(ClimberConstants.SERVO_ENABLED_ANGLE);
	}

	/**
	 * Disengages the ratchet.
	 */
	private void disableRatchet() {
		rachetServo.setAngle(ClimberConstants.SERVO_DISABLED_ANGLE);
	}

	/**
	 * Disengages the ratchet and raises the climber.
	 *
	 * @return
	 *         Command to run.
	 */
	public Command RaiseClimber(double position) {
		return Commands.runOnce(() -> {
			disableRatchet();
			climberMotor.set(ClimberConstants.RAISE_CLIMBER_SPEED);
		}, this)
				.andThen(Commands.waitUntil(() -> (climberAbsoluteEncoder.getPosition() >= position)))
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
		return Commands.runOnce(() -> {
			enableRatchet();
			climberMotor.set(ClimberConstants.LOWER_CLIMBER_SPEED);
		}, this)
				.andThen(Commands.idle(this))
				.finallyDo(() -> {
					climberMotor.set(0);
				});
	}
}
