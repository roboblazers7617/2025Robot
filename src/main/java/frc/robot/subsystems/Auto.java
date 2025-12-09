package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.text.ParseException;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import io.github.roboblazers7617.limelight.PoseEstimate;

/**
 * Subsystem for the robot's autonomous functionality.
 */
public class Auto {
	private static PathPlannerPath lastRunPath = null; // stores the most recently run path

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
		// store the path for later referencing
		try {
			lastRunPath = PathPlannerPath.fromPathFile(pathName);
		} catch (IOException e) {
			lastRunPath = null;
			System.out.println("no auto path was loaded");
		} catch (org.json.simple.parser.ParseException e) {
			lastRunPath = null;
			System.out.println("path json could not be parsed");
		} catch (FileVersionException e) {
			lastRunPath = null;
			System.out.println("path json could not be parsed");
		}

		// Create a path following command using AutoBuilder. This will also trigger event markers.
		// TODO: #119 (Max) I think would be better to add the ResetLastAngularScalar here
		return new PathPlannerAuto(pathName);
	}

	/*
	 * creates a path from the robots last run path end position(determined by the lastRunPath variable) to the transform3D and returns it
	 */
	public static PathPlannerPath createPathFromTransform(Transform3d transform, Drivetrain drivetrain) {
		Pose2d startPose2d;
		Pose2d currentPose2d = drivetrain.getPose();
		if (lastRunPath == null) {
			// if no lastRunPath is found, use the current start position
			System.out.println("No previous auto path was found, using current position");
			startPose2d = currentPose2d;
		} else {
			// if the path exists, get the end pose of it
			startPose2d = lastRunPath.getPathPoses().get(lastRunPath.getPathPoses().size() - 1);
		}
		// take the transform3d and turn in into a pose2d and add current position to get to global coordinates
		Pose2d endPose2d = new Pose2d(transform.getX() + currentPose2d.getX(), transform.getY() + currentPose2d.getY(), transform.getRotation().toRotation2d().plus(currentPose2d.getRotation()));
		List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose2d, endPose2d);

		// copy the constraints of the previous path
		PathConstraints constraints = lastRunPath.getGlobalConstraints();

		PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, null);
		path.preventFlipping = true;

		return path;
	}
}
