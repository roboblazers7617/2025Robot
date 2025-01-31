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

@Logged

public class Climber extends SubsystemBase {
	private final SparkMax rampPivot = new SparkMax(Constants.ClimberConstants.RAMP_PIVOT_PORT, MotorType.kBrushless);
	private final RelativeEncoder rampPivotEncoder;

	/** Creates a new Climber. */
	public Climber() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.idleMode(IdleMode.kBrake);

		rampPivot.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
		rampPivotEncoder = rampPivot.getEncoder();
		rampPivotEncoder.setPosition(0.0);
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
	 * Set the velocity of the ramp pivot motor.
	 * 
	 * @param rampPivotSpeed
	 *            The speed to set. Value should be between -1.0 and 1.0.
	 */
	public void setSpeedRampPivot(double rampPivotSpeed) {
		rampPivot.set(rampPivotSpeed);
	}

	/**
	 * Get the position of the ramp pivot motor.
	 * 
	 * @return Number the position of the motor
	 */
	public double getPositionRampPivot() {
		return rampPivotEncoder.getPosition();
	}

	@Override
	public void periodic() {}
}