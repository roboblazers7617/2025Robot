package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Subsystem for the robot's autonomous functionality.
 */
public class Auto {
	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public static void setupPathPlanner(Drivetrain drivetrain, DriverStation.Alliance alliance) {
		// Load the RobotConfig from the GUI settings. You should probably
		// store this in your Constants file
		RobotConfig config;
		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = false;
			// Configure AutoBuilder last
			AutoBuilder.configure(drivetrain::getPose,
					// Robot pose supplier
					drivetrain::resetOdometry,
					// Method to reset odometry (will be called if your auto has a starting pose)
					drivetrain::getRobotVelocity,
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							drivetrain.getSwerveDrive().drive(speedsRobotRelative, drivetrain.getSwerveDrive().kinematics.toSwerveModuleStates(speedsRobotRelative), moduleFeedForwards.linearForces());
						} else {
							drivetrain.getSwerveDrive().setChassisSpeeds(speedsRobotRelative);
						}
					},
					// Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
					new PPHolonomicDriveController(
							// PPHolonomicController is the built in path following controller for holonomic drive trains
							AutoConstants.TRANSLATION_PID_CONSTANTS, // Translation PID constants
							AutoConstants.ROTATION_PID_CONSTANTS // Rotation PID constants
					), config,
					// The robot configuration
					() -> {
						// Boolean supplier that controls when the path will be mirrored for the red alliance
						// This will flip the path being followed to the red side of the field.
						// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

						return alliance == DriverStation.Alliance.Red;
						// return true;
					}, drivetrain
			// Reference to this subsystem to set requirements
			);
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		// Preload PathPlanner Path finding
		// IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
		PathfindingCommand.warmupCommand().schedule();
	}

	/**
	 * Configures AutoBuilder if it hasn't already been configured. This should be run on enable so things like pathfinding commands don't cause the code to crash.
	 *
	 * @param drivetrain
	 *            The Drivetrain for PathPlanner to control.
	 */
	public static void setupPathPlannerFailsafe(Drivetrain drivetrain) {
		if (!AutoBuilder.isConfigured()) {
			System.err.println("AutoBuilder not configured before enabling! Configuring AutoBuilder with data from the FMS.");
			Auto.setupPathPlanner(drivetrain, DriverStation.getAlliance()
					.orElse(DriverStation.Alliance.Blue));
		}
	}

	/**
	 * Get the path follower with events.
	 *
	 * @param pathName
	 *            PathPlanner path name.
	 * @return
	 *         {@link AutoBuilder#followPath(PathPlannerPath)} path command.
	 */
	public static Command getAutonomousCommand(String pathName) {
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		// TODO: #119 (Max) I think would be better to add the ResetLastAngularScalar here
		return new PathPlannerAuto(pathName);
	}
}
