// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class StubbedCommands {
	public class Drivetrain extends SubsystemBase {
		public static Command DisableVision() {
			return Commands.print("Disable Vision");
		}

		public static Command LockWheels() {
			return Commands.print("Lock Wheels");
		}

		public static Command DriverFastMode() {
			return Commands.print("Fast Mode");
		}

		public static Command DriverSlowMode() {
			return Commands.print("Slow Mode");
		}

		/**
		 * Aligns the robot to the left of the tag closest to it.
		 * <p>
		 * <ul>
		 * <li>Aligning to the left Branch of the Reef face closest</li>
		 * <li>Aligning to the leftmost section of the closest coral station</li>
		 * <li>Align to leftmost cage for climbing</li>
		 * <li>Align to center of Processor</li>
		 * </ul>
		 */
		public static Command AlignLeftOfTag() {
			return Commands.print("Align Left of Tag");
		}

		/**
		 * Aligns the robot to the right of the tag closest to it.
		 * <p>
		 * <ul>
		 * <li>Aligning to the right Branch of the Reef face closest</li>
		 * <li>Aligning to the rightmost section of the closest coral station</li>
		 * <li>Align to rightmost cage for climbing</li>
		 * <li>Align to center of Processor</li>
		 * </ul>
		 */
		public static Command AlignRightOfTag() {
			return Commands.print("Align Right of Tag");
		}

		/**
		 * Aligns the robot to the middle of the tag closest to it.
		 * <p>
		 * <ul>
		 * <li>Aligning to remove Algae from the Reef face closest</li>
		 * <li>Aligning to the center section of the closest coral station</li>
		 * <li>Align to center cage for climbing</li>
		 * <li>Align to center of Processor</li>
		 * </ul>
		 */
		public static Command AlignMiddleOfTag() {
			return Commands.print("Align Left of Tag");
		}
	}

	public class Climber extends SubsystemBase {
		public static Command StowRamp() {
			return Commands.print("Stow Ramp");
		}

		public static Command RampDown() {
			return Commands.print("Ramp Down");
		}

		public static Command RampUp() {
			return Commands.print("Ramp Up");
		}

		public static Command AutoClimb() {
			return Commands.print("Auto Climb");
		}

		public static Command ClimberDown() {
			return Commands.print("Climber Down");
		}
	}

	public class Elevator extends SubsystemBase {
		public static Command MoveElevatorAndWristManual(Supplier<Double> elevatorControllerInput, Supplier<Double> wristControllerInput) {
			return Commands.print("Move Elevator and Wrist Manually").andThen(Commands.idle((Subsystem[]) null));
		}

		public static Command MoveL4() {
			return Commands.print("Move to Position to place L4 Coral");
		}

		public static Command MoveL3() {
			return Commands.print("Move to Position to place L3 Coral");
		}

		public static Command MoveL2() {
			return Commands.print("Move to Position to place L2 Coral");
		}

		public static Command MoveL1() {
			return Commands.print("Move to Position to place L1 Coral");
		}

		public static Command MoveProcessor() {
			return Commands.print("Move to Position to place Algae in Processor");
		}

		public static Command StowCoral() {
			return Commands.print("Move to stow position for Coral");
		}

		public static Command StowAlgae() {
			return Commands.print("Move to stow position for Algae");
		}

		public static Command MoveIntakeCoral() {
			return Commands.print("Move to Position to intake Coral from Coral Station");
		}

		public static Command MoveHighAlgae() {
			return Commands.print("Move to Position to intake High Algae");
		}

		public static Command MoveLowAlgae() {
			return Commands.print("Move to Position to intake Low Algae");
		}
	}

	public class EndEffector extends SubsystemBase {
		public static Command IntakeCoral() {
			return Commands.print("Run automation to intake Coral");
		}

		public static Command IntakeAlgae() {
			return Commands.print("Run automation to intake Algae");
		}

		public static Command OutakeCoral() {
			return Commands.print("Run automation to outake Coral");
		}

		public static Command OutakeAlgae() {
			return Commands.print("Run automation to outake Algae");
		}

		/**
		 * Returns true if the robot is currently holding an algae
		 */
		public static boolean isHoldingAlage() {
			return false;
		}
	}
}
