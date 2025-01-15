package frc.robot;

import frc.robot.generated.VersionConstantsGenerated;

/**
 * Constants that contain version metadata for the robot. Wraps an auto-generated class to add Javadoc and logging.
 */
public class VersionConstants {
	/**
	 * Maven group which the robot package belongs to.
	 */
	public static final String MAVEN_GROUP = VersionConstantsGenerated.MAVEN_GROUP;

	/**
	 * Maven package name of the robot package.
	 */
	public static final String MAVEN_NAME = VersionConstantsGenerated.MAVEN_NAME;

	/**
	 * Robot code version.
	 */
	public static final String VERSION = VersionConstantsGenerated.VERSION;

	/**
	 * Git revision number.
	 */
	public static final int GIT_REVISION = VersionConstantsGenerated.GIT_REVISION;

	/**
	 * Git commit hash.
	 */
	public static final String GIT_SHA = VersionConstantsGenerated.GIT_SHA;

	/**
	 * Git commit date.
	 */
	public static final String GIT_DATE = VersionConstantsGenerated.GIT_DATE;

	/**
	 * Git branch.
	 */
	public static final String GIT_BRANCH = VersionConstantsGenerated.GIT_BRANCH;

	/**
	 * Date on which the robot program was built in a human-readable format.
	 */
	public static final String BUILD_DATE = VersionConstantsGenerated.BUILD_DATE;

	/**
	 * Date on which the robot program was built in unix time.
	 */
	public static final long BUILD_UNIX_TIME = VersionConstantsGenerated.BUILD_UNIX_TIME;

	/**
	 * Are there uncommited changes?
	 */
	public static final int DIRTY = VersionConstantsGenerated.DIRTY;
}
