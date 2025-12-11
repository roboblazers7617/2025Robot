package frc.robot.subsystems.vision;

import java.util.List;

import org.opencv.core.Mat;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import pabeles.concurrency.IntOperatorTask.Min;

public class PhotonVision {
	private PhotonCamera camera;
	private final boolean DEBUG_MODE = true; // do print statements work or not
	private PhotonPipelineResult latestResult;

	/*
	 * The FOV of the camera, used for calculating object distance
	 */
	private final double DIAGONAL_FOV = 70.0;
	private final double HORIZONTAL_FOV;
	private final double VERTICAL_FOV;

	private final double ASPECT_RATIO;

	private final double CAMERA_RESOLUTION_HORIZONTAL = 800;
	private final double CAMERA_RESOLUTION_VERTICAL = 600;

	public static enum OBJECT_ID {
		CORAL, ALGAE
	}

	public PhotonVision(String cameraName) {
		camera = new PhotonCamera(cameraName);
		ASPECT_RATIO = CAMERA_RESOLUTION_HORIZONTAL / CAMERA_RESOLUTION_VERTICAL;

		HORIZONTAL_FOV = Math.toDegrees(2 * Math.atan(Math.tan(DIAGONAL_FOV / 2) * ASPECT_RATIO / Math.sqrt(Math.pow(ASPECT_RATIO, 2) + 1)));
		VERTICAL_FOV = Math.toDegrees(2 * Math.atan(Math.tan(DIAGONAL_FOV / 2) * 1 / Math.sqrt(Math.pow(ASPECT_RATIO, 2) + 1)));

		if (DEBUG_MODE) {
			System.out.println("Photoncamera created with camera name " + cameraName);
			System.out.println("Diagonal FOV: " + DIAGONAL_FOV);
			System.out.println("Horizontal FOV: " + HORIZONTAL_FOV);
			System.out.println("Vertical FOV: " + VERTICAL_FOV);
		}
	}

	/**
	 * updates the latestResult variable and returns it.
	 * this function is basically a less efficient replacement for PhotonCamera.getLatestResult() because that function is deprecated
	 */
	private PhotonPipelineResult updateLatestResult() {
		List<PhotonPipelineResult> results = camera.getAllUnreadResults();
		// if there are new results, update the latestResult, otherwise leave it as is
		if (results.size() != 0) {
			latestResult = results.get(results.size() - 1);
		}
		return latestResult;
	}

	/**
	 * returns the transform3d to the closest object of the specified objectID
	 * returns null if none is found
	 */
	public Transform3d LookForClosestTarget(OBJECT_ID objectID) {
		if (DEBUG_MODE) {
			System.out.println("Looking for the closest tracked target of objectID " + objectID.name() + "/" + objectID.ordinal());
		}

		double closestTransformDistance = 99999;
		Transform3d closestTransform3d = null;

		PhotonPipelineResult result = updateLatestResult();
		for (int idx = 0; idx < result.targets.size(); idx++) {
			PhotonTrackedTarget target = result.targets.get(idx);
			target.de;
			// Check if the target is of the correct objectID
			if (target.objDetectId != objectID.ordinal()) {
				if (DEBUG_MODE) {
					System.out.println("Target of ID " + target.objDetectId + " was discarded for not matching specified objectID " + objectID);
				}
				continue;
			}

			// Check the distance of the transform and compare to the closest
			double distance = Math.sqrt(Math.pow(target.bestCameraToTarget.getX(), 2) + Math.pow(target.bestCameraToTarget.getY(), 2) + Math.pow(target.bestCameraToTarget.getZ(), 2));

			if (distance < closestTransformDistance) {
				closestTransformDistance = distance;
				closestTransform3d = target.bestCameraToTarget;
				if (DEBUG_MODE) {
					System.out.println("Target was " + distance + " away which was larger than " + closestTransformDistance + " and closest transform is now " + closestTransform3d);
				}
			} else if (DEBUG_MODE) {
				System.out.println("Target was " + distance + " away which was smaller than " + closestTransformDistance);
			}
		}
		if (DEBUG_MODE) {
			if (closestTransform3d == null) {
				System.out.println("No targets were seen");
			} else {
				System.out.println("the closest target was " + closestTransform3d);
			}
		}
		return closestTransform3d;
	}

	private Transform3d getTransform3dFromBoundingBox(List<TargetCorner> corners) {
		// the object height and width, in inches
		double object_width = 4;
		double object_height = 12;

		double minX = Math.min(Math.min(corners.get(0).x, corners.get(1).x), Math.min(corners.get(3).x, corners.get(4).x));
		double maxX = Math.max(Math.max(corners.get(0).x, corners.get(1).x), Math.max(corners.get(3).x, corners.get(4).x));

		double minY = Math.min(Math.min(corners.get(0).y, corners.get(1).y), Math.min(corners.get(3).y, corners.get(4).y));
		double maxY = Math.max(Math.max(corners.get(0).y, corners.get(1).y), Math.max(corners.get(3).y, corners.get(4).y));

		double boundingBoxWidthPixels = maxX - minX;
		double boundingBoxHeightPixels = maxY - minY;

		double areaPercentOfBoundingBox = (boundingBoxHeightPixels * boundingBoxWidthPixels) / (CAMERA_RESOLUTION_HORIZONTAL * CAMERA_RESOLUTION_VERTICAL);

		// find the center of the bounding box
		double boundingBoxCenterX = (maxX + minX) * .5;
		double boundingBoxCenterY = (maxY + minY) * .5;

		// find the rotation of the rectangular object by reverse engineering from the ration of y/x
		double thetaOfRectangularObject = Math.atan((boundingBoxWidthPixels * object_width - boundingBoxHeightPixels * object_height) / (boundingBoxHeightPixels * object_width - boundingBoxWidthPixels * object_height));

		// find what proportions the bounding box has
		double boundingBoxXRelativeWidth = object_height * Math.cos(thetaOfRectangularObject) + object_width * Math.sin(thetaOfRectangularObject);
		double boundingBoxXRelativeHeight = object_width * Math.cos(thetaOfRectangularObject) + object_height * Math.sin(thetaOfRectangularObject);

		// calculate the proportion of a bounding box rotated by theta compared to the bounding box if the object were flat
		double areaProportionToNormalized = (boundingBoxXRelativeHeight * boundingBoxXRelativeWidth) / (object_height * object_width);
		// use this proportion to calculate the approxamate area a flat object would take up at the same distance
		double areaPercentageNormalized = areaPercentOfBoundingBox / areaProportionToNormalized;

		// calculate the distance to the object
		double distanceToObject = calculateDistanceToObject(object_width, object_height, areaPercentageNormalized);

		// calculate the proportion from -1 to 1 of the objects center position on the screen
		double xProportional = (boundingBoxCenterX - CAMERA_RESOLUTION_HORIZONTAL * .5) / (.5 * CAMERA_RESOLUTION_HORIZONTAL);
		double yProportional = (boundingBoxCenterY - CAMERA_RESOLUTION_VERTICAL * .5) / (.5 * CAMERA_RESOLUTION_VERTICAL);

		// find the angle around the z(up and down) axis to the object
		double thetaToObject = .5 * xProportional * HORIZONTAL_FOV;

		// find the angle around the y(left and right) axis to the object
		double phiToObject = .5 * yProportional * VERTICAL_FOV;

		// find the x, y, and z coordinates in spherical, phi is not the drop angle like it is in standard spherical models, so sin(phi) and cos(phi) are swapped from standard conversions
		double xCoordinate = distanceToObject * Math.cos(thetaToObject) * Math.cos(phiToObject);
		double yCoordinate = distanceToObject * Math.sin(thetaToObject) * Math.cos(phiToObject);
		double zCoordinate = distanceToObject * Math.sin(phiToObject);

		return new Transform3d(xCoordinate, yCoordinate, zCoordinate, new Rotation3d());
	}

	/*
	 * Calculates the distance to an object given the percentage of the total are it occupies, as well as its dimensions
	 */
	private double calculateDistanceToObject(double object_width, double object_height, double areaPercentage) {
		// TODO (Riley) Implement when you have more sleep
		return 0;
	}
}
