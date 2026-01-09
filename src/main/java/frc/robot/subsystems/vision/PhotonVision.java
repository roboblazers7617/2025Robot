package frc.robot.subsystems.vision;

import java.util.List;
import java.util.Map;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.IntOperatorTask.Min;

public class PhotonVision extends SubsystemBase {
	private final PhotonCamera camera;

	public static enum OBJECTS {
		ALGAE, CORAL
	}

	/*
	 * the dimensions of each object in the following order
	 * [isSpherical (1 if true, 0 if false), objectHeight, objectWidth, objectDepth]
	 */
	private final Map<OBJECTS, double[]> OBJECT_DIMENSIONS = Map.of(OBJECTS.ALGAE, new double[] { 1, 16.25, 16.25, 16.25 }, OBJECTS.CORAL, new double[] { 0, 12, 4.5, 4.5 });

	/*
	 * The FOV of the camera, used for calculating object distance
	 */
	private final double DIAGONAL_FOV = 70.0;
	private final double HORIZONTAL_FOV;
	private final double VERTICAL_FOV;

	private final double ASPECT_RATIO;

	private final double CAMERA_RESOLUTION_HORIZONTAL = 800;
	private final double CAMERA_RESOLUTION_VERTICAL = 600;

	/*
	 * the a in ax+b of the linear regression model for finding pixels to distance
	 */
	private final double LINEAR_REGRESSION_DISTANCE_A = 0.0014515;
	/*
	 * the b in ax+b of the linear regression model for finding pixels to distance
	 */
	private final double LINEAR_REGRESSION_DISTANCE_B = -0.00043953;

	public PhotonVision(String cameraName) {
		camera = new PhotonCamera(cameraName);
		if (!camera.isConnected()) {
			// throw new Error("camera could not be connected");
		}
		ASPECT_RATIO = CAMERA_RESOLUTION_HORIZONTAL / CAMERA_RESOLUTION_VERTICAL;

		HORIZONTAL_FOV = Math.toDegrees(2 * Math.atan(Math.tan(DIAGONAL_FOV / 2) * ASPECT_RATIO / Math.sqrt(Math.pow(ASPECT_RATIO, 2) + 1)));
		VERTICAL_FOV = Math.toDegrees(2 * Math.atan(Math.tan(DIAGONAL_FOV / 2) * 1 / Math.sqrt(Math.pow(ASPECT_RATIO, 2) + 1)));
		System.out.println("Photoncamera created with camera name " + cameraName);
		System.out.println("Diagonal FOV: " + DIAGONAL_FOV);
		System.out.println("Horizontal FOV: " + HORIZONTAL_FOV);
		System.out.println("Vertical FOV: " + VERTICAL_FOV);
	}

	public Transform3d getTransformOfObject(OBJECTS objectID) {
		PhotonPipelineResult result = camera.getLatestResult();
		Transform3d transformResult;
		if (result.hasTargets()) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			for (int idx = 0; idx < targets.size(); idx++) {
				PhotonTrackedTarget target = targets.get(idx);
				if (target.getDetectedObjectClassID() != objectID.ordinal()) {
					System.out.println("detected object of ID " + target.getDetectedObjectClassID() + " does not match selected objectID of " + objectID.ordinal());
					continue;
				}
				transformResult = getTransform3dFromBoundingBox(target.getMinAreaRectCorners(), objectID);
				System.out.println("target is transformed by" + transformResult);
				// TODO this needs to look for the closest target, not just the first one it sees, but for now this will work
				return transformResult;
			}
		} else {
			// System.out.println("no targets");
		}

		return null;
	}

	/*
	 * takes in the corners of a bounding box and generates the transform from the camera to the object, the rotation is always 0
	 */
	private Transform3d getTransform3dFromBoundingBox(List<TargetCorner> corners, OBJECTS objectID) {
		if (corners.size() != 4) {
			// System.out.println("only "+corners.size()+" corners are given");
			return new Transform3d();
		} else {
			System.out.println("enough corners, proceeding");
		}

		/*
		 * should only be true if all dimensions are the same and the object is a sphere
		 * this stops rotation detection, as the program assumes that an object is a cube, which can mess with rotation
		 */
		boolean isSpherical;
		if (OBJECT_DIMENSIONS.get(objectID)[0] >= 1) {
			isSpherical = true;
		} else {
			isSpherical = false;
		}

		// the object height, depth and width, in inches
		double object_height = OBJECT_DIMENSIONS.get(objectID)[1];
		double object_width = OBJECT_DIMENSIONS.get(objectID)[2];
		double object_depth = OBJECT_DIMENSIONS.get(objectID)[3];

		double minX = Math.min(Math.min(corners.get(0).x, corners.get(1).x), Math.min(corners.get(2).x, corners.get(3).x));
		double maxX = Math.max(Math.max(corners.get(0).x, corners.get(1).x), Math.max(corners.get(2).x, corners.get(3).x));

		double minY = Math.min(Math.min(corners.get(0).y, corners.get(1).y), Math.min(corners.get(2).y, corners.get(3).y));
		double maxY = Math.max(Math.max(corners.get(0).y, corners.get(1).y), Math.max(corners.get(2).y, corners.get(3).y));

		double boundingBoxWidthPixels = maxX - minX;
		double boundingBoxHeightPixels = maxY - minY;

		System.out.println("bounding box dimensions are " + boundingBoxWidthPixels + " by " + boundingBoxHeightPixels);

		// find the center of the bounding box
		double boundingBoxCenterX = (maxX + minX) * .5;
		double boundingBoxCenterY = (maxY + minY) * .5;

		// translate the pixel values to estimated length in inches using the ratio of the height, this assumes that no matter what orientation the height of the object is the
		double boundingBoxEstimatedWidthInches = boundingBoxWidthPixels * (object_height / boundingBoxHeightPixels);

		// find what proportions the bounding box has relative to what it should be when rotation = 0
		// if the object is spherical, the proportion just = 1 because rotation causes no change in bounding box dimnsions
		double translationProportionToNormalize = 1;

		// using the information given you can only determine one of 2 possible orientations, for now I have decided to just comment it out
		// if (!isSpherical){
		// // find the rotation of the rectangular object around the z axis by reverse engineering from the ratio of y/x
		// double thetaOfRectangularObject;

		// //test 2 candidates using theta = atan(w/l) +- arccos(v/(sqrt(l^2+w^2)))
		// //candidate 1 will be correct when theta < atan(w/l) and candidate 2 will be correct when theta > atan(w/l)
		// double innerTerm = Math.min(1, boundingBoxEstimatedWidthInches/(Math.sqrt(Math.pow(object_width, 2)+Math.sqrt(Math.pow(object_depth, 2)))));
		// double thetaCandidate1 = Math.atan(object_depth/object_width) - Math.acos(innerTerm);
		// double thetaCandidate2 = Math.atan(object_depth/object_width) + Math.acos(innerTerm);

		// System.out.println("theta "+thetaCandidate1+" "+thetaCandidate2);
		// System.out.println(boundingBoxEstimatedWidthInches/(Math.sqrt(Math.pow(object_width, 2)+Math.sqrt(Math.pow(object_depth, 2)))));
		// //decide the correct candidate
		// double candidate1PredictionError = Math.abs((object_width * Math.cos(thetaCandidate1)+object_depth * Math.sin(thetaCandidate1)) - boundingBoxEstimatedWidthInches);
		// double candidate2PredictionError = Math.abs((object_width * Math.cos(thetaCandidate2)+object_depth * Math.sin(thetaCandidate2)) - boundingBoxEstimatedWidthInches);
		// if (candidate1PredictionError < candidate2PredictionError){
		// thetaOfRectangularObject = thetaCandidate1;
		// } else{
		// thetaOfRectangularObject = thetaCandidate2;
		// }
		// translationProportionToNormalize = object_width / Math.abs(object_width * Math.cos(thetaOfRectangularObject)+object_depth * Math.sin(thetaOfRectangularObject));
		// }

		double boundingBoxNormalizedWidthPixels = boundingBoxWidthPixels * translationProportionToNormalize;
		System.out.println("normalized bounding box dimensions are " + boundingBoxNormalizedWidthPixels + " by " + boundingBoxHeightPixels);
		// calculate the distance to the object
		// if the object is not spherical, only use the height to estimate distance
		double distanceToObject;
		if (isSpherical) {
			distanceToObject = calculateDistanceToObject(object_width, object_height, boundingBoxNormalizedWidthPixels, boundingBoxHeightPixels);
		} else {
			distanceToObject = calculateDistanceToObject(object_height, boundingBoxHeightPixels);
		}

		System.out.println("bounding box is centered at " + boundingBoxCenterX + " , " + boundingBoxCenterY);
		// calculate the proportion from -1 to 1 of the objects center position on the screen
		double xProportional = (boundingBoxCenterX - CAMERA_RESOLUTION_HORIZONTAL * .5) / (.5 * CAMERA_RESOLUTION_HORIZONTAL);
		double yProportional = (boundingBoxCenterY - CAMERA_RESOLUTION_VERTICAL * .5) / (.5 * CAMERA_RESOLUTION_VERTICAL);
		System.out.println("bounding box is proportionally centered at " + xProportional + " , " + yProportional);
		// find the angle around the z(up and down) axis to the object
		double thetaToObject = .5 * xProportional * HORIZONTAL_FOV;

		// find the angle around the y(left and right) axis to the object
		double phiToObject = .5 * yProportional * VERTICAL_FOV;

		// convert to radians
		thetaToObject *= Math.PI / 180;
		phiToObject *= Math.PI / 180;

		System.out.println("object is located at Rho: " + distanceToObject + ", Theta: " + thetaToObject + ", Phi: " + phiToObject);
		// find the x, y, and z coordinates in spherical, unlike standard spherical models,phi =0 at the center of the screen(up is negative, down is positive) so sin(phi) and cos(phi) are swapped from standard conversions
		double xCoordinate = distanceToObject * Math.cos(thetaToObject) * Math.cos(phiToObject);
		double yCoordinate = distanceToObject * Math.sin(thetaToObject) * Math.cos(phiToObject);
		double zCoordinate = distanceToObject * -Math.sin(phiToObject);

		return new Transform3d(xCoordinate, yCoordinate, zCoordinate, new Rotation3d());
	}

	/*
	 * Calculates the distance to an object in inches by using a linreg equation to estimate distance based on width and height, then average the 2
	 * linereg distance a and b will need to be recalibrated for each model of camera. this was for a ov9782,
	 * to recalibrate, take one game piece, preferably round, then measure the amt of pixels it takes up on screen, divide by diameter, and record distance, then use a linreg where x = pixels/diameter and y = 1/distance
	 */
	private double calculateDistanceToObject(double objectWidth, double objectHeight, double widthPixels, double heightPixels) {
		double distanceWidth = calculateDistanceToObject(objectWidth, widthPixels);
		double distanceHeight = calculateDistanceToObject(objectHeight, heightPixels);

		System.out.println(" the error between distances was " + (distanceHeight - distanceWidth));
		System.out.println("average distance was calculated to be " + ((distanceHeight + distanceWidth) / 2));
		return ((distanceWidth + distanceHeight) / 2);
	}

	private double calculateDistanceToObject(double sizeInches, double sizePixels) {
		return (1 / (LINEAR_REGRESSION_DISTANCE_A * sizePixels / sizeInches + LINEAR_REGRESSION_DISTANCE_B));
	}

	@Override
	public void periodic() {
		// getTransformOfObject(OBJECTS.ALGAE);
	}

	public Command getPhotonCamLookCommand() {
		return this.runOnce(() -> getTransformOfObject(OBJECTS.ALGAE));
	}
}
