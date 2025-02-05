// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

/**
 * The elevator subsystem.
 * Elevator and wrist safety features are in the {@link #periodic()} method.
 */
public class Elevator extends SubsystemBase {
	private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KV, ElevatorConstants.KA);
	/** The right motor. */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** The left motor. */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

	/** The elevator target in meters, this may not be safe, if no value than it is in speed controll. */
	private double elevatorTarget = 0;

	private final ArmFeedforward wristFeedforward = new ArmFeedforward(WristConstants.KS, WristConstants.KG, WristConstants.KV);

	/**
	 * the wrist
	 */
	private final SparkMax wristMotor = new SparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);

	/** the wrist target in degrees, this may not be safe, if no value than it is in speed controll */
	private double wristTarget = 0;

	/**
	 * This does something to do with the elevator.
	 */
	public Elevator() {
		SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();

		baseElevatorConfig.idleMode(IdleMode.kBrake);
		baseElevatorConfig.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
		// baseElevatorConfig.closedLoopRampRate(0.5); TODO I've added current limits and break mode. The only other things I found were these but I don't think we want them.
		// baseElevatorConfig.voltageCompensation(12); or this

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

		leaderElevatorMotor.configure(baseElevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig().apply(baseElevatorConfig).follow(leaderElevatorMotor); // TODO I could not find any documentation on this and I'm not sure if I just use the base config and add a follow if that could effect the leader motor.
		followerElevatorMotor.configure(followerElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkMaxConfig wristConfig = new SparkMaxConfig();
		wristConfig.idleMode(IdleMode.kBrake);
		wristConfig.smartCurrentLimit(WristConstants.CURRENT_LIMIT);
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
		// TODO: (Brandon) Where is the logic for upper bounds control? The outer limits are managed in the set position function. The lower areas where they may collide is handled here.

		// This function is responsible for they safety of the elevator and wrist.
		// It uses to elevatorTarget and wristTarget to determine the target position of the elevator and wrist. If the elevator and wrist are both below their SAFE_MIN_POSITION they will collide.
		// Whenever either one is below their SAFE_MIN_POSITION the other will be limited to the SAFE_MIN_POSITION.

		// if there is no elevator target, do nothing (this is probably because the elevator is being controlled by a speed)
		// TODO: (Brandon) What ensures that elevator / wrist are in safe positions if they are being controlled by speed rather than position? The speed function will now just call set position and will therefore have all the same safety stuff.
		// ensure elevator target is within bounds
		double safeElevatorTarget = elevatorTarget;

		// ensure elevator target is not too low if the wrist is low
		if (wristMotor.getEncoder().getPosition() < WristConstants.SAFE_MIN_POSITION && safeElevatorTarget < ElevatorConstants.SAFE_MIN_POSITION) {
			safeElevatorTarget = ElevatorConstants.SAFE_MIN_POSITION;
		}
		double elevatorFeedForwardValue = elevatorFeedforward.calculate(5); // TODO
		leaderElevatorMotor.getClosedLoopController().setReference(safeElevatorTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, elevatorFeedForwardValue); // TODO ensure slot is correct

		double safeWristTarget = wristTarget;

		// ensure wrist target is not too low if the elevator is low
		if (leaderElevatorMotor.getEncoder().getPosition() < ElevatorConstants.SAFE_MIN_POSITION && safeWristTarget < WristConstants.SAFE_MIN_POSITION) {
			safeWristTarget = WristConstants.SAFE_MIN_POSITION;
		}
		double wristFeedForwardValue = wristFeedforward.calculate(5, 5); // TODO
		wristMotor.getClosedLoopController().setReference(safeWristTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, wristFeedForwardValue); // TODO ensure slot is correct
	}

	/**
	 * A command set the elvator speed in m/s.
	 * 
	 * @param speed
	 * @return the command
	 */
	public Command setElevatorSpeedCommand(DoubleSupplier speed) {
		return null; // TODO
	}

	/**
	 * A command set the wrist speed in degrees/s.
	 * 
	 * @param speed
	 *            the speed in degrees/s
	 * @return the command
	 */
	// TODO: (Brandon) Is any speed valid? Should it be? The PID controller has a max velocity built in, that is not something we need to handle.
	public Command setWristSpeedCommand(DoubleSupplier speed) {
		return null; // TODO
	}

	/**
	 * A command to move the wrist to a position in degrees.
	 * 
	 * @param position
	 * @return the command
	 */
	// TODO: (Brandon) Why is this a double supplier? If it wasn't a supplier only the value at startup would be used.
	// TODO: (Brandon) Is any position double valid? How does this relate to the usage of the mechanism? Position is validaded in the setWristPosition function. I don't get your second question.
	public Command setWristPositionCommand(DoubleSupplier position) {
		return this.runOnce(() -> {
			setWristPosition(position.getAsDouble());
		});
	}

	/**
	 * A function to move the elevator to a position in meters.
	 * 
	 * @param position
	 *            the position in meters
	 */
	// TODO: (Brandon) Is any position valid? How do you equate a position as a double with a task such as "intake". Position is now validaded. This function would be called by commands like "intake" which would supply the position.
	private void setElevatorPosition(double position) {
		MathUtil.clamp(position, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION);
		elevatorTarget = position;
	}

	/**
	 * A function to move the elevator to a position in degrees.
	 * 
	 * @param position
	 *            the position in degrees
	 */
	// TODO: (Brandon) Is any position valid? How do you equate a position as a double with a task such as "intake". Position is now validaded. This function would be called by commands like "intake" which would supply the position.
	private void setWristPosition(double position) {
		MathUtil.clamp(position, WristConstants.MIN_POSITION, WristConstants.MAX_POSITION);
		wristTarget = position;
	}

	// TODO: (Brandon) You have basic commands. Where are the ones that will be tied to buttons for controlled movements to move both wrist and elevator together? How do you move to set positions?
}
