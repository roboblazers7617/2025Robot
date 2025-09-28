// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.RobotContainer;

@Logged
/**
 * Subsystem to control the elevator and wrist.
 * <p>
 * Elevator and wrist safety features are in the {@link #periodic()} method.
 */
public class Elevator extends SubsystemBase {
	@NotLogged
	private final RobotContainer robotContainer;
	/**
	 * The FeedForward used for the elevator.
	 */
	private final ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(ElevatorConstants.KS, ElevatorConstants.KG, ElevatorConstants.KV);
	/**
	 * The right elevator motor.
	 */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	/**
	 * The left elevator motor.
	 */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

	// private final SparkAbsoluteEncoder elevatorAbsoluteEncoder;

	private final RelativeEncoder leaderElevatorRelativeEncoder;

	@Logged
	private final RelativeEncoder followerElevatorRelativeEncoder;

	private final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new Constraints(ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_ACCELERATION));

	/**
	 * This is the current trapezoid profile setpoint for the elevator. It is not the final target, that is in the {@link #elevatorTarget} variable.
	 */
	private TrapezoidProfile.State currentElevatorSetpoint;

	/**
	 * The elevator target in meters, This is within the outer bounds of the elevator but the danger zone at the bottom has not been accounted for.
	 */
	@Logged
	private double elevatorTarget = 0;

	/**
	 * The FeedForward used for the wrist.
	 */
	private final ArmFeedforward wristFeedforward = new ArmFeedforward(WristConstants.KS, WristConstants.KG, WristConstants.KV);
	/**
	 * The wrist motor.
	 */
	private final SparkMax wristMotor = new SparkMax(WristConstants.MOTOR_ID, MotorType.kBrushless);

	private final AbsoluteEncoder wristAbsoluteEncoder;

	private final TrapezoidProfile wristProfile = new TrapezoidProfile(new Constraints(WristConstants.MAX_VELOCITY, WristConstants.MAX_ACCELERATION));

	/**
	 * This is the current trapezoid profile setpoint for the wrist. It is not the final target, that is in the {@link #wristTarget} variable.
	 */
	private TrapezoidProfile.State currentWristSetpoint;

	/**
	 * The wrist target in degrees, This is within the outer bounds of the wrist but the danger zone at the bottom has not been accounted for.
	 */
	private double wristTarget;

	/**
	 * Creates a new Elevator.
	 */
	public Elevator(RobotContainer robotContainer) {
		this.robotContainer = robotContainer;

		SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();

		baseElevatorConfig.idleMode(IdleMode.kBrake);
		baseElevatorConfig.smartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
		baseElevatorConfig.inverted(false);
		// ramp rate?

		baseElevatorConfig.encoder
				.positionConversionFactor(ElevatorConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(ElevatorConstants.VELOCITY_CONVERSION_FACTOR);

		leaderElevatorRelativeEncoder = leaderElevatorMotor.getEncoder();
		followerElevatorRelativeEncoder = followerElevatorMotor.getEncoder();
		currentElevatorSetpoint = new TrapezoidProfile.State(leaderElevatorRelativeEncoder.getPosition(), 0);

		baseElevatorConfig.closedLoop
				.p(ElevatorConstants.KP)
				.i(ElevatorConstants.KI)
				.d(ElevatorConstants.KD)
				.outputRange(ElevatorConstants.KMIN_OUTPUT, ElevatorConstants.KMAX_OUTPUT);
		// .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

		// baseElevatorConfig.closedLoop.maxMotion
		// .maxVelocity(ElevatorConstants.MAX_VELOCITY)
		// .maxAcceleration(ElevatorConstants.MAX_ACCELERATION);

		leaderElevatorMotor.configure(baseElevatorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkBaseConfig followerElevatorMotorConfig = new SparkMaxConfig()
				.apply(baseElevatorConfig)
				.follow(leaderElevatorMotor, true);
		followerElevatorMotor.configure(followerElevatorMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkMaxConfig wristConfig = new SparkMaxConfig();
		wristConfig.idleMode(IdleMode.kBrake);
		wristConfig.smartCurrentLimit(WristConstants.CURRENT_LIMIT);
		wristConfig.inverted(true);

		wristConfig.absoluteEncoder
				.positionConversionFactor(360.0)
				.velocityConversionFactor(360 * 60)
				.zeroOffset(WristConstants.ZERO_OFFSET)
				.inverted(true)
				.zeroCentered(true);
		wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();

		wristConfig.encoder
				.positionConversionFactor(WristConstants.POSITION_CONVERSION_FACTOR)
				.velocityConversionFactor(WristConstants.VELOCITY_CONVERSION_FACTOR);
		// wristEncoder = wristMotor.getEncoder();
		// wristEncoder.setPosition(wristAbsoluteEncoder.getPosition());

		wristConfig.closedLoop
				.p(WristConstants.KP)
				.i(WristConstants.KI)
				.d(WristConstants.KD)
				.outputRange(WristConstants.KMIN_OUTPUT, WristConstants.KMAX_OUTPUT)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		// wristConfig.closedLoop.maxMotion
		// .maxVelocity(WristConstants.MAX_VELOCITY)
		// .maxAcceleration(WristConstants.MAX_ACCELERATION);

		wristMotor.configure(wristConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		setWristPosition(wristAbsoluteEncoder.getPosition());
		currentWristSetpoint = new TrapezoidProfile.State(wristTarget, 0);

		SmartDashboard.putData("Toggle Elevator Brake Mode", toggleBrakeModesCommand());
		SmartDashboard.putData("Stop Elevator Command", doNothing());
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
		if (wristAbsoluteEncoder.getPosition() < WristConstants.SAFE_MIN_POSITION && safeElevatorTarget < ElevatorConstants.MAX_LOWERED_POSITION) {
			safeElevatorTarget = ElevatorConstants.MAX_LOWERED_POSITION;
			// System.out.println("elevator block 1 ");
		}
		// ensure the elevator target is not too high if wrist is high (wrist collides with metal connecter thing on top of the robot)
		if (wristAbsoluteEncoder.getPosition() > WristConstants.SAFE_MAX_POSITION && safeElevatorTarget > ElevatorConstants.MAX_LOWERED_POSITION) {
			safeElevatorTarget = ElevatorConstants.MAX_LOWERED_POSITION;
			// System.out.println("elevator block 2 ");
		}

		currentElevatorSetpoint = elevatorProfile.calculate(0.02, currentElevatorSetpoint, new TrapezoidProfile.State(safeElevatorTarget, 0));
		if (Math.abs(currentElevatorSetpoint.position - safeElevatorTarget) < 0.01) {
			// System.out.println("At target");
		}

		double elevatorFeedForwardValue = elevatorFeedforward.calculate(currentElevatorSetpoint.velocity); // this is technically supposed to be the velocity setpoint

		leaderElevatorMotor.getClosedLoopController().setReference(currentElevatorSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, elevatorFeedForwardValue, ArbFFUnits.kVoltage);

		//
		//
		//

		double safeWristTarget = wristTarget;

		// ensure wrist target is not too low if the elevator is low
		if (leaderElevatorRelativeEncoder.getPosition() < ElevatorConstants.MAX_LOWERED_POSITION && safeWristTarget < WristConstants.SAFE_MIN_POSITION) {
			safeWristTarget = WristConstants.SAFE_MIN_POSITION;
			// System.out.println("wrist block 1 ");
		}

		// ensure wrist target is not too high if the elevator is high
		if (leaderElevatorRelativeEncoder.getPosition() > ElevatorConstants.MAX_LOWERED_POSITION && safeWristTarget > WristConstants.SAFE_MAX_POSITION) {
			safeWristTarget = WristConstants.SAFE_MAX_POSITION;
			// System.out.println("wrist block 2 ");
		}

		// ensure wrist target is not too high (a different too high, a smaller one) if it is holding algae
		if (safeWristTarget > WristConstants.MAX_ALGAE_POSITION_WITH_ELEVATOR && robotContainer.isHoldingAlgae()) {
			safeWristTarget = WristConstants.MAX_ALGAE_POSITION_WITH_ELEVATOR;
		}

		currentWristSetpoint = wristProfile.calculate(0.02, currentWristSetpoint, new TrapezoidProfile.State(safeWristTarget, 0));
		// System.out.println("safe wrist target: " + safeWristTarget);
		// System.out.println("current wrist setpoint: " + currentWristSetpoint.position);
		double wristFeedForwardValue = wristFeedforward.calculate(Math.toRadians(currentWristSetpoint.position), Math.toRadians(currentWristSetpoint.velocity));

		wristMotor.getClosedLoopController().setReference(currentWristSetpoint.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, wristFeedForwardValue, ArbFFUnits.kVoltage);
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
			private boolean isElevatorJoystickCentered = false;
			private boolean isWristJoystickCentered = false;

			@Override
			public void execute() {
				double targetElevatorSpeed = MathUtil.clamp(elevatorSpeed.getAsDouble() * ElevatorConstants.MAX_VELOCITY, -ElevatorConstants.MAX_VELOCITY, ElevatorConstants.MAX_VELOCITY);

				if (targetElevatorSpeed != 0) {
					setElevatorPosition(elevatorTarget + (targetElevatorSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
					isElevatorJoystickCentered = false;
				} else if (!isElevatorJoystickCentered) {
					setElevatorPosition(leaderElevatorRelativeEncoder.getPosition());
					currentElevatorSetpoint = new TrapezoidProfile.State(leaderElevatorRelativeEncoder.getPosition(), 0);
					isElevatorJoystickCentered = true;
				}

				double targetWristSpeed = MathUtil.clamp(wristSpeed.getAsDouble() * WristConstants.MAX_VELOCITY, -WristConstants.MAX_VELOCITY, WristConstants.MAX_VELOCITY);
				if (targetWristSpeed != 0) {
					setWristPosition(wristTarget + (targetWristSpeed / 50)); // divide the speed by 50 because their are 50 loops per second
					isWristJoystickCentered = false;
				} else if (!isWristJoystickCentered) {
					setWristPosition(wristAbsoluteEncoder.getPosition());
					currentWristSetpoint = new TrapezoidProfile.State(wristAbsoluteEncoder.getPosition(), 0);
					isWristJoystickCentered = true;
				}
			}

			@Override
			public boolean isFinished() {
				return false;
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
	public Command SetPositionCommand(Constants.ArmPosition position) {
		Command command = new Command() {
			@Override
			public void initialize() {
				setElevatorPosition(position.ELEVATOR_POSITION);
				setWristPosition(position.WRIST_POSITION);
			}

			@Override
			public boolean isFinished() {
				return isAtTarget();
			}

			@Override
			public void end(boolean interrupted) {
				System.out.println("done. interrupted: " + interrupted);
			}
		};
		command.addRequirements(this);

		return command;
	}

	/**
	 * A command that does nothing. This command requires the elevator subsystem so it will kill any other command. This is used to let the drivers regain controll of the elevator if it is unable to reach its target.
	 *
	 * @return Command to run.
	 */
	public Command doNothing() {
		Command command = Commands.none();
		command.addRequirements(this);
		return command;
	}

	public double getElevatorTarget() {
		return elevatorTarget;
	}

	public void elevatorInit() {
		setWristPosition(wristAbsoluteEncoder.getPosition());
		currentWristSetpoint = new TrapezoidProfile.State(wristAbsoluteEncoder.getPosition(), 0);
		setElevatorPosition(leaderElevatorRelativeEncoder.getPosition());
		currentElevatorSetpoint = new TrapezoidProfile.State(leaderElevatorRelativeEncoder.getPosition(), 0);
	}

	/**
	 * A function to move the elevator to a position in meters.
	 *
	 * @param position
	 *            The position in meters.
	 */
	private void setElevatorPosition(double position) {
		elevatorTarget = MathUtil.clamp(position, ElevatorConstants.MIN_POSITION, ElevatorConstants.MAX_POSITION);
		;
	}

	/**
	 * A function to move the elevator to a position in degrees.
	 *
	 * @param position
	 *            The position in degrees.
	 */
	private void setWristPosition(double position) {
		wristTarget = MathUtil.clamp(position, WristConstants.MIN_POSITION, WristConstants.MAX_POSITION);
	}

	/**
	 * Check if the elevator and wrist are within the tolerance to their target positions. This is used to determine if the {@link #SetPositionCommand(Constants.ArmPosition)} command is finished.
	 */
	private boolean isAtTarget() {
		boolean elevatorAtTarget = Math.abs(leaderElevatorRelativeEncoder.getPosition() - elevatorTarget) < ElevatorConstants.TOLERANCE;
		boolean wristAtTarget = Math.abs(wristAbsoluteEncoder.getPosition() - wristTarget) < WristConstants.TOLERANCE;
		return elevatorAtTarget && wristAtTarget;
	}

	/**
	 * Toggles brake mode on the elevator and wrist motors.
	 *
	 * @return
	 *         Command to run.
	 */
	private Command toggleBrakeModesCommand() {
		return this.runOnce(() -> {
			SparkBaseConfig newElevatorConfig = new SparkMaxConfig();
			if (leaderElevatorMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newElevatorConfig.idleMode(IdleMode.kCoast);
			} else {
				newElevatorConfig.idleMode(IdleMode.kBrake);
			}
			leaderElevatorMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
			followerElevatorMotor.configure(newElevatorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

			SparkBaseConfig newWristConfig = new SparkMaxConfig();
			if (wristMotor.configAccessor.getIdleMode() == IdleMode.kBrake) {
				newWristConfig.idleMode(IdleMode.kCoast);
			} else {
				newWristConfig.idleMode(IdleMode.kBrake);
			}
			wristMotor.configure(newWristConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
		}).ignoringDisable(true);
	}
}
