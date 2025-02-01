// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.commands.WaitUntilInterrupt;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged

public class EndEffector extends SubsystemBase {
	/** Creates a new EndEffector. */
	private final SparkMax endEffectorMotor = new SparkMax(EndEffectorConstants.CAN_ID_END_EFFECTOR, MotorType.kBrushless);
	private final DigitalInput coralBeamBreak = new DigitalInput(EndEffectorConstants.BEAM_BREAK_DIO);

	public EndEffector() {
		SparkBaseConfig motorConfig = new SparkMaxConfig()
				.smartCurrentLimit(40)
				.idleMode(IdleMode.kBrake)
				.apply(EndEffectorConstants.CLOSED_LOOP_CONFIG);
		motorConfig.encoder.positionConversionFactor(EndEffectorConstants.POSITION_CONVERSION_FACTOR);

		endEffectorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("End Effector Velocity", endEffectorMotor.getEncoder().getVelocity());
	}

	private void stopMotor() {
		endEffectorMotor.set(0);
	}

	private void startMotor(Double speed) {
		endEffectorMotor.set(speed);
	}

	/**
	 * Should be able to use multiple speeds for the motors bassed on the buttons?
	 * this may be wrong though.
	 * 
	 * @param speed
	 *            Speed to set [-1,1].
	 * @return
	 *         Command to run.
	 */
	public Command StartMotor(Supplier<Double> speed) {
		return this.runOnce(() -> {
			startMotor(speed.get()); /* one-time action goes here */
		});
	}

	/**
	 * Stops the intake motor.
	 * 
	 * @return
	 *         Command to run.
	 */
	public Command StopMotor() {
		return this.runOnce(() -> {
			stopMotor();
		});
	}

	public Command CoralIntake() {
		return StartMotor(() -> EndEffectorConstants.CORAL_INTAKE_SPEED),(new WaitUntilInterrupt(coralBeamBreak, (rising, falling) -> {
			stopMotor();
		}, false, true));
	}

	public Command CoralOuttake() {
		return StartMotor(() -> EndEffectorConstants.CORAL_OUTAKE_SPEED)
				.andThen(Commands.waitUntil(() -> !coralBeamBreak.get()))
				.andThen(Commands.waitSeconds(EndEffectorConstants.WAIT_TIME))
				.andThen(StopMotor());
	}

	public Command AlgaeIntake() {
		return null;
	}

	/*
	 * 
	 */
	public Command AlgaeOuttake() {
		return null;
	}

	/**
	 * Tests the beam break sensor by printing to standard output when an interrupt is triggered.
	 * 
	 * @return
	 *         Command to run.
	 */
	public Command TestBeamBreak() {
		return new WaitUntilInterrupt(coralBeamBreak, (rising, falling) -> {
			if (rising) {
				System.out.println("Interrupt triggered on the rising edge.");
			}

			if (falling) {
				System.out.println("Interrupt triggered on the falling edge.");
			}
		});
	}
}
