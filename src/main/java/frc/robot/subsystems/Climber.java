// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Subsystem that controls the climber.
 */
@Logged
public class Climber extends SubsystemBase {
	private final SparkMax rightClimber = new SparkMax(Constants.ClimberConstants.RIGHT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder rightClimberEncoder;

	private final SparkMax leftClimber = new SparkMax(Constants.ClimberConstants.LEFT_CLIMBER_PORT, MotorType.kBrushless);
	private final RelativeEncoder leftClimberEncoder;

	private final SparkMax rampPivot = new SparkMax(Constants.ClimberConstants.RAMP_PIVOT_PORT, MotorType.kBrushless);
	private final RelativeEncoder rampPivotEncoder;

	/**
	 * Creates a new Climber.
	 */
	public Climber() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake);
		// TODO: make sure the correct motors are inverted
		leftClimber.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rightClimber.configure(motorConfig.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		leftClimberEncoder = leftClimber.getEncoder();
		rightClimberEncoder = rightClimber.getEncoder();
		leftClimberEncoder.setPosition(0.0);
		rightClimberEncoder.setPosition(0.0);

		rampPivot.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		rampPivotEncoder = rampPivot.getEncoder();
	}

	/**
	 * sets the speeds of the climb motors
	 *
	 * @param leftSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 * @param rightSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void setSpeed(double leftSpeed, double rightSpeed) {
		// leftClimber.set(speed);
		rightClimber.set(rightSpeed);
		leftClimber.set(leftSpeed);
	}

	/**
	 * Get the velocity of the right climb motor.
	 *
	 * @return Number the RPM of the motor
	 */
	public double getSpeedRight() {
		return rightClimberEncoder.getVelocity();
	}

	/**
	 * Get the velocity of the left climb motor.
	 *
	 * @return Number the RPM of the motor
	 */
	public double getSpeedLeft() {
		return leftClimberEncoder.getVelocity();
	}

	/**
	 * Get the velocity of the ramp pivot motor.
	 *
	 * @return Number the RPM of the motor
	 */
	public double getSpeedRampPivot() {
		return rampPivotEncoder.getVelocity();
	}

	/**
	 * Set the velocity of the left climb motor.
	 *
	 * @param leftSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void setSpeedLeft(double leftSpeed) {
		leftClimber.set(leftSpeed);
	}

	/**
	 * Set the velocity of the right climb motor.
	 *
	 * @param rightSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void setSpeedRight(double rightSpeed) {
		rightClimber.set(rightSpeed);
	}

	/**
	 * Set the velocity of the ramp pivot motor.
	 *
	 * @param rampPivotSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void setSpeedRampPivot(double rampPivotSpeed) {
		rampPivot.set(rampPivotSpeed);
	}

	/**
	 * Get the position of the right climb motor. This returns the native units of 'rotations' by default
	 *
	 * @return Number of rotations of the motor
	 */
	public double getPositionRightMotor() {
		return rightClimberEncoder.getPosition();
	}

	/**
	 * Get the position of the left climb motor. This returns the native units of 'rotations'
	 *
	 * @return Number of rotations of the motor
	 */
	public double getPositionLeftMotor() {
		return leftClimberEncoder.getPosition();
	}

	@Override
	public void periodic() {}
}
