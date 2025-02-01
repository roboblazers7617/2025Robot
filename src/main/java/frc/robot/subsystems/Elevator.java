// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Reef;
import frc.robot.Constants.WristConstants;

public class Elevator extends SubsystemBase {
	/** The right motor */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** The left motor */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

	/** the elevator target in meters, this may not be safe */
	private Optional<Double> elevatorTarget = Optional.of(0.0);

	/**
	 * the wrist
	 */
	private final SparkMax wristMotor = new SparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);

	/** the wrist target in degrees, this may not be safe */
	private Optional<Double> wristTarget = Optional.of(0.0);

	public Elevator() {
		SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();

		baseElevatorConfig.absoluteEncoder
				.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
				.zeroOffset(ElevatorConstants.ZERO_OFFSET);

		baseElevatorConfig.closedLoop
				.p(ElevatorConstants.KP)
				.i(ElevatorConstants.KI)
				.d(ElevatorConstants.KD)
				.outputRange(ElevatorConstants.KMIN_OUTPUT, ElevatorConstants.KMAX_OUTPUT)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		baseElevatorConfig.closedLoop.maxMotion
				.maxVelocity(ElevatorConstants.MAX_VELOCITY)
				.maxAcceleration(ElevatorConstants.MAX_ACCELERATION);

		SparkBaseConfig leaderElevatorMotorConfig = new SparkMaxConfig().apply(baseElevatorConfig);
		leaderElevatorMotor.configure(leaderElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig().apply(baseElevatorConfig).follow(leaderElevatorMotor);
		followerElevatorMotor.configure(followerElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkMaxConfig wristConfig = new SparkMaxConfig();

		wristConfig.absoluteEncoder
				.positionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR)
				.zeroOffset(WristConstants.ZERO_OFFSET);

		wristConfig.closedLoop
				.p(WristConstants.KP)
				.i(WristConstants.KI)
				.d(WristConstants.KD)
				.outputRange(WristConstants.KMIN_OUTPUT, WristConstants.KMAX_OUTPUT)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		wristConfig.closedLoop.maxMotion
				.maxVelocity(WristConstants.MAX_VELOCITY)
				.maxAcceleration(WristConstants.MAX_ACCELERATION);

		wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run

		// if there is no elevator target, do nothing (this is probably because the elevator is being controlled by a speed)
		if (elevatorTarget.isPresent()) {
			// ensure elevator target is within outer most bounds
			double target = elevatorTarget.get();
			if (target < ElevatorConstants.MIN_POSITION) {
				target = ElevatorConstants.MIN_POSITION;
			} else if (target > ElevatorConstants.MAX_POSITION) {
				target = ElevatorConstants.MAX_POSITION;
			}

			// ensure elevator target is not too low if the wrist is low
			if (wristMotor.getEncoder().getPosition() < WristConstants.SAFE_MIN_POSITION && target < ElevatorConstants.SAFE_MIN_POSITION) {
				target = ElevatorConstants.SAFE_MIN_POSITION;
			}
		}

		// if there is no wrist target, do nothing (this is probably because the wrist is being controlled by a speed)
		if (wristTarget.isPresent()) {
			// ensure wrist target is within outer most bounds
			double target = wristTarget.get();
			if (target < WristConstants.MIN_POSITION) {
				target = WristConstants.MIN_POSITION;
			} else if (target > WristConstants.MAX_POSITION) {
				target = WristConstants.MAX_POSITION;
			}

			// ensure wrist target is not too low if the elevator is low
			if (leaderElevatorMotor.getEncoder().getPosition() < ElevatorConstants.SAFE_MIN_POSITION && target < WristConstants.SAFE_MIN_POSITION) {
				target = WristConstants.SAFE_MIN_POSITION;
			}
		}
	}

	private void setElevatorPosition(double position) {
		// leaderElevatorMotor.getClosedLoopController().setReference(position, ControlType.kPosition);
		elevatorTarget = Optional.of(position);
	}

	private void setElevatorSpeed(double speed) {
		leaderElevatorMotor.getClosedLoopController().setReference(speed, ControlType.kVelocity);
		elevatorTarget = Optional.empty();
	}

	/**
	 * a command set the elvator speed in m/s
	 * 
	 * @param speed
	 * @return the command
	 */
	public Command setElevatorSpeedCommand(DoubleSupplier speed) {
		return this.runOnce(() -> {
			setElevatorSpeed(speed.getAsDouble());
		});
	}

	/**
	 * a command to move the elevator to l1
	 */
	public Command setElevator(Reef reef) {
		return this.runOnce(() -> {
			setElevatorPosition(reef.getHeight());
		});
	}
}
