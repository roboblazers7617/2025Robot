// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.rmi.server.ServerCloneException;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class Climber extends SubsystemBase {
	private final SparkMax climbMotor = new SparkMax(Constants.ClimberConstants.CLIMB_MOTOR_PORT, MotorType.kBrushless);
	private final RelativeEncoder climbMotorEncoder;

	private final Servo climbRachet = new Servo(Constants.ClimberConstants.CLIMB_SERVO_PORT);
	private double servoPosition = 0.0;

	/** Creates a new Climber. */
	public Climber() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake);

		climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		climbMotorEncoder = climbMotor.getEncoder();
		climbMotorEncoder.setPosition(0.0);
	}

	/**
	 * Get the velocity of the rachet motor.
	 * 
	 * @return Number the RPM of the motor
	 */
	public double getSpeedClimbMotor() {
		return climbMotorEncoder.getVelocity();
	}

	/**
	 * Set the velocity of the rachet motor.
	 * 
	 * @param climbMotorSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */

	/**
	 * Set the position of the servo.
	 * 
	 * @param position
	 *            Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
	 */
	public void setServoPosition(double position) {
		// climbRachet.set(position);
		climbRachet.set(position);
	}

	/**
	 * Get the position of the rachet motor.
	 * 
	 * @return Number the position of the motor
	 */
	public double getPositionClimbMotor() {
		return climbMotorEncoder.getPosition();
	}

	/**
	 * Get the position of the servo.
	 * 
	 * @return Servo values range from 0.0 to 1.0 corresponding to the range of full left to full right.
	 */

	public double getServoPosition() {
		return climbRachet.get();
	}

	@Override
	public void periodic() {
		servoPosition = getServoPosition();
	}
}