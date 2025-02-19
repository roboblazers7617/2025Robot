// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

/**
 * Subsystem to control the elevator and wrist.
 * <p>
 * Elevator and wrist safety features are in the {@link #periodic()} method.
 */
public class Elevator extends SubsystemBase {
	/**
	 * The FeedForward used for the elevator.
	 */
	private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	/**
	 * The right elevator motor.
	 */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/**
	 * The left elevator motor.
	 */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

	private final SparkAbsoluteEncoder elevatorEncoder;

	/**
	 * The elevator target in meters, This is within the outer bounds of the elevator but the danger zone at the bottom has not been accounted for.
	 */
	private double elevatorTarget = 0;

	/**
	 * The FeedForward used for the wrist.
	 */
	private final ArmFeedforward wristFeedforward = new ArmFeedforward(WristConstants.KS, WristConstants.KG, WristConstants.KV);
	/**
	 * The wrist motor.
	 */
	private final SparkMax wristMotor = new SparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);

	private final SparkAbsoluteEncoder wristEncoder;

	/**
	 * The wrist target in degrees, This is within the outer bounds of the wrist but the danger zone at the bottom has not been accounted for.
	 */
	private double wristTarget = 0;

	/**
	 * Creates a new Elevator.
	 */
	public Elevator() {
		SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();

		baseElevatorConfig.idleMode(IdleMode.kBrake);
		baseElevatorConfig.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);

		baseElevatorConfig.absoluteEncoder
				.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR)
				.zeroOffset(ElevatorConstants.ZERO_OFFSET);
		elevatorEncoder = leaderElevatorMotor.getAbsoluteEncoder();

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

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig().apply(baseElevatorConfig).follow(leaderElevatorMotor);
		followerElevatorMotor.configure(followerElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkMaxConfig wristConfig = new SparkMaxConfig();
		wristConfig.idleMode(IdleMode.kBrake);
		wristConfig.smartCurrentLimit(WristConstants.CURRENT_LIMIT);
		wristConfig.absoluteEncoder
				.positionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR)
				.zeroOffset(WristConstants.ZERO_OFFSET);
		wristEncoder = wristMotor.getAbsoluteEncoder();

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

	/**
	 * This method is responsible for they safety of the elevator and wrist.
	 * <p>
	 * This uses the {@link #elevatorTarget} and {@link #wristTarget} to determine the target position of the elevator and wrist. If the elevator and wrist are both below their SAFE_MIN_POSITION they will collide. Whenever either one is below their SAFE_MIN_POSITION the other will be limited to the SAFE_MIN_POSITION.
	 */
	@Override
	public void periodic() {
		// ensure elevator target is within bounds
		double safeElevatorTarget = elevatorTarget;

		// ensure elevator target is not too low if the wrist is low
		if (wristEncoder.getPosition() < WristConstants.SAFE_MIN_POSITION && safeElevatorTarget < ElevatorConstants.SAFE_MIN_POSITION) {
			safeElevatorTarget = ElevatorConstants.SAFE_MIN_POSITION;
		}
		double elevatorFeedForwardValue = elevatorFeedforward.calculate(elevatorEncoder.getVelocity()); // this is technically supposed to be the velocity setpoint
		leaderElevatorMotor.getClosedLoopController().setReference(safeElevatorTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, elevatorFeedForwardValue, ArbFFUnits.kVoltage);

		double safeWristTarget = wristTarget;

		// ensure wrist target is not too low if the elevator is low
		if (elevatorEncoder.getPosition() < ElevatorConstants.SAFE_MIN_POSITION && safeWristTarget < WristConstants.SAFE_MIN_POSITION) {
			safeWristTarget = WristConstants.SAFE_MIN_POSITION;
		}

		double wristFeedForwardValue = wristFeedforward.calculate(Math.toRadians(elevatorEncoder.getPosition()), Math.toRadians(elevatorEncoder.getVelocity()));
		wristMotor.getClosedLoopController().setReference(safeWristTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, wristFeedForwardValue, ArbFFUnits.kVoltage);
	}

	/**
	 * A command to set the speeds of the elevator and wrist.
	 * 
	 * @param elevatorSpeed
	 *            The speed of the elevator as a percentage of max speed. [-1, 1]
	 * @param wristSpeed
	 *            The speed of the wrist as a percentage of max speed. [-1, 1]
	 * @return
	 *         {@link Command} to run.
	 */
	public Command setSpeedsCommand(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
		Command command = new Command() {
			@Override
			public void execute() {
				double targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble() * ElevatorConstants.MAX_VELOCITY, -ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_VELOCITY);

				setElevatorPosition(elevatorTarget + (targetElevatorSpeed / 50)); // divide the speed by 50 because their are 50 loops per second

				double targetWristSpeed = MathUtil.clamp(wristSpeed.getAsDouble(), -1, 1) * WristConstants.MAX_VELOCITY;
				setWristPosition(wristTarget + (targetWristSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
			}
		};
		command.addRequirements(this);
		return command;
	}

	/**
	 * A command to set the elevator and wrist to a position.
	 * 
	 * @param position
	 *            The Constants.ArmPosition to set the elevator and wrist to.
	 * @return
	 *         {@link Command} to run.
	 */
	public Command SetPosition(Constants.ArmPosition position) {
		Command command = new InstantCommand(() -> {
			setElevatorPosition(position.ELEVATOR_POSITION);
			setWristPosition(position.WRIST_POSITION);
		});
		command.addRequirements(this);
		return command;
	}

	/**
	 * A function to move the elevator to a position in meters.
	 *
	 * @param position
	 *            The position in meters.
	 */
	private void setElevatorPosition(double position) {
		MathUtil.clamp(position, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION);
		elevatorTarget = position;
	}

	/**
	 * A function to move the elevator to a position in degrees.
	 *
	 * @param position
	 *            The position in degrees.
	 */
	private void setWristPosition(double position) {
		MathUtil.clamp(position, WristConstants.MIN_POSITION, WristConstants.MAX_POSITION);
		wristTarget = position;
	}
}
