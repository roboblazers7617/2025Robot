package frc.robot.subsystems.vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVision {
	private PhotonCamera camera;
	private final boolean DEBUG_MODE = true; // do print statements work or not
	private PhotonPipelineResult latestResult;

	public PhotonVision(String cameraName) {
		camera = new PhotonCamera(cameraName);
		if (DEBUG_MODE) {
			System.out.println("Photoncamera created with camera name " + cameraName);
		}
	}

	// updates the latestResult variable and returns it
	// this function is basically a less efficient replacement for PhotonCamera.getLatestResult() because that function is deprecated
	private PhotonPipelineResult updateLatestResult() {
		List<PhotonPipelineResult> results = camera.getAllUnreadResults();
		// if there are new results, update the latestResult, otherwise leave it as is
		if (results.size() != 0) {
			latestResult = results.get(results.size() - 1);
		}
		return latestResult;
	}

	// returns the transform3d to the closest object of the specified objectID
	// returns null if none is found
	// TODO change objectID to an enum
	public Transform3d LookForClosestTarget(int objectID) {
		if (DEBUG_MODE) {
			System.out.println("Looking for the closest tracked target of objectID " + objectID);
		}

		double closestTransformDistance = 99999;
		Transform3d closestTransform3d = null;

		PhotonPipelineResult result = updateLatestResult();
		for (int idx = 0; idx < result.targets.size(); idx++) {
			PhotonTrackedTarget target = result.targets.get(idx);

			// Check if the target is of the correct objectID
			if (target.objDetectId != objectID) {
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
}
