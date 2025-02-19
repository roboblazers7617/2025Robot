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
		// baseElevatorConfig.closedLoopRampRate(0.5); TODO I've added current limits and break mode. The only other things I found were these but I don't think we want them.
		// baseElevatorConfig.voltageCompensation(12); or this

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

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig().apply(baseElevatorConfig).follow(leaderElevatorMotor); // TODO I could not find any documentation on this and I'm not sure if I just use the base config and add a follow if that could effect the leader motor.
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
		// TODO: (Brandon) Where is the logic for upper bounds control? The outer limits are managed in the set position function. The lower areas where they may collide is handled here.

		// ensure elevator target is within bounds
		double safeElevatorTarget = elevatorTarget;

		// ensure elevator target is not too low if the wrist is low
		if (wristEncoder.getPosition() < WristConstants.SAFE_MIN_POSITION && safeElevatorTarget < ElevatorConstants.SAFE_MIN_POSITION) {
			safeElevatorTarget = ElevatorConstants.SAFE_MIN_POSITION;
		}
		double elevatorFeedForwardValue = elevatorFeedforward.calculate(elevatorEncoder.getVelocity()); // this is technically supposed to be the velocity setpoint
		// TODO: #128 Pass in Units for Elevator FF
		leaderElevatorMotor.getClosedLoopController().setReference(safeElevatorTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, elevatorFeedForwardValue, ArbFFUnits.kVoltage);

		double safeWristTarget = wristTarget;

		// ensure wrist target is not too low if the elevator is low
		if (elevatorEncoder.getPosition() < ElevatorConstants.SAFE_MIN_POSITION && safeWristTarget < WristConstants.SAFE_MIN_POSITION) {
			safeWristTarget = WristConstants.SAFE_MIN_POSITION;
		}
		// TODO: #130 Control upper bound of wrist

		double wristFeedForwardValue = wristFeedforward.calculate(Math.toRadians(elevatorEncoder.getPosition()), Math.toRadians(elevatorEncoder.getVelocity()));
		// TODO: #129 Pass in Units for Wrist FF
		wristMotor.getClosedLoopController().setReference(safeWristTarget, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, wristFeedForwardValue, ArbFFUnits.kVoltage);
	}

	/**
	 * A command to set the speeds of the elevator and wrist.
	 * 
	 * @param elevatorSpeed
	 *            The speed of the elevator as a percentage of max speed. [-1, 1]
	 * @param wristSpeed
	 *            The speed of the wrist as a percentage of max speed. [-1, 1] //TODO #127
	 * @return
	 *         {@link Command} to run.
	 */
	public Command setSpeedsCommand(DoubleSupplier elevatorSpeed, DoubleSupplier wristSpeed) {
		Command command = new Command() {
			@Override
			public void execute() {
				// double targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble(), -1, 1) * ElevatorConstants.MAX_VELOCITY;
				double targetElevatorSpeed;
				if (elevatorSpeed.getAsDouble() >= 0) {
					targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble(), 0, 1) * ElevatorConstants.MAX_VELOCITY;
				} else {
					targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble(), -1, 0) * ElevatorConstants.MIN_VELOCITY;
				}
				setElevatorPosition(elevatorTarget + (targetElevatorSpeed / 50)); // divide the speed by 50 because their are 50 loops per second

				double targetWristSpeed = MathUtil.clamp(wristSpeed.getAsDouble(), -1, 1) * WristConstants.MAX_VELOCITY;
				setWristPosition(wristTarget + (targetWristSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
			}
		};
		command.addRequirements(this);
		return command;
	}

	/**
	 * A command set the elvator speed in m/s.
	 * 
	 * @deprecated use {@link #setSpeedsCommand(DoubleSupplier, DoubleSupplier)} instead.
	 * @param speed
	 *            The speed in m/s.
	 * @return
	 *         {@link Command} to run.
	 */
	// TODO: #125 Change units of parameter
	public Command setElevatorSpeedCommand(DoubleSupplier speed) {
		Command command = new Command() {
			@Override
			public void execute() {
				// TODO: #124 Check for Min_velocity as well (if going backward)
				double targetSpeed = speed.getAsDouble();
				if (targetSpeed > ElevatorConstants.MAX_VELOCITY) {
					targetSpeed = ElevatorConstants.MAX_VELOCITY;
				}
				setElevatorPosition(elevatorTarget + (targetSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
			}
		};
		// command.addRequirements(this); TODO it seems like the subsystem should be required but that would prevent both speed commands from being used at the same time.
		// also if there is lag after they release the joystick we may need to reset the position target to the current position.
		return command;
	}

	/**
	 * A command set the wrist speed in degrees/s.
	 * * @deprecated use {@link #setSpeedsCommand(DoubleSupplier, DoubleSupplier)} instead.
	 * 
	 * @param speed
	 *            The speed in degrees/s.
	 * @return
	 *         {@link Command} to run.
	 */
	// TODO: #126 Change Units for Parameter for setWristSpeedCommand
	public Command setWristSpeedCommand(DoubleSupplier speed) {
		Command command = new Command() {
			@Override
			public void execute() {
				// TODO: #127 Check for minimum value for wrist speed
				double targetSpeed = speed.getAsDouble();
				if (targetSpeed > WristConstants.MAX_VELOCITY) {
					targetSpeed = WristConstants.MAX_VELOCITY;
				}
				setWristPosition(wristTarget + (targetSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
			}
		};
		// command.addRequirements(this); TODO it seems like the subsystem should be required but that would prevent both speed commands from being used at the same time.
		// also if there is lag after they release the joystick we may need to reset the position target to the current position.
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
	// TODO: (Brandon) Is any position valid? How do you equate a position as a double with a task such as "intake". Position is now validaded. This function would be called by commands like "intake" which would supply the position.
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
	// TODO: (Brandon) Is any position valid? How do you equate a position as a double with a task such as "intake". Position is now validaded. This function would be called by commands like "intake" which would supply the position.
	private void setWristPosition(double position) {
		MathUtil.clamp(position, WristConstants.MIN_POSITION, WristConstants.MAX_POSITION);
		wristTarget = position;
	}

	// TODO: (Brandon) You have basic commands. Where are the ones that will be tied to buttons for controlled movements to move both wrist and elevator together? How do you move to set positions?
}
