// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorConstants;

public class Arm extends SubsystemBase {
	private final SparkMax leaderArmMotor = new SparkMax(ArmConstants.LEFT_MOTOR_ID, MotorType.kBrushless);
	private final SparkMax followerArmMotor = new SparkMax(ArmConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);

	private final SparkAbsoluteEncoder armAbsoluteEncoder = leaderArmMotor.getAbsoluteEncoder();
	private final SparkClosedLoopController armPIDController = leaderArmMotor.getClosedLoopController();

	/** The right motor */
	private final SparkMax leaderElevatorMotor = new SparkMax(ElevatorConstants.RIGHT_MOTOR_ID, MotorType.kBrushless);
	/** The left motor */
	private final SparkMax followerElevatorMotor = new SparkMax(ElevatorConstants.LEFT_MOTOR_ID, MotorType.kBrushless);

	/** Creates a new Arm. */
	public Arm() {
		SparkMaxConfig baseArmConfig = new SparkMaxConfig();

		baseArmConfig.closedLoop
				.p(ArmConstants.KP)
				.i(ArmConstants.KI)
				.d(ArmConstants.KD)
				.outputRange(ArmConstants.KMIN_OUTPUT, ArmConstants.KMAX_OUTPUT)
				.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

		baseArmConfig.closedLoop.maxMotion
				.maxVelocity(ArmConstants.MAX_VELOCITY)
				.maxAcceleration(ArmConstants.MAX_ACCELERATION);

		SparkBaseConfig leaderArmMotorConfig = new SparkMaxConfig().apply(baseArmConfig);
		leaderArmMotor.configure(leaderArmMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);

		SparkBaseConfig followerArmMotorConfig = new SparkMaxConfig().apply(baseArmConfig).follow(leaderArmMotor);
		followerArmMotor.configure(followerArmMotorConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kPersistParameters);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
