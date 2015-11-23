import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

import lejos.robotics.navigation.Pose;

public class PathPlanner {
	// Use VFH+

	// Goal orientation
	// If height of next step in the goal orientation direction exceeds height band, while not approaching goal,
	// add delta to orientation to keep within height band
	
	// Need timestep -> freq of refresh rate

	public Pose goal;
	public Pose myPose;
	public Pose mylastPose;
	public Boolean finalApproach;
	public float projDist; // dt
	
	// Coefficients
	public float angRes = 5; // angular resolution of histogram
	public float FULLROT = 360; // depends on if degrees or radians

	// weight values for cost function
	public int[] m = {5, 2, 2};

	public PathPlanner(Pose currentPose, Pose lastPose, Pose destination) {
		// TODO Auto-generated constructor stub
		// ANGLES ARE ALL IN DEGREES
		myPose = currentPose;
		mylastPose = lastPose;
		goal = destination;
		finalApproach = Boolean.FALSE;
	}
	
	public ArrayList<Float> calculateCandidateDir(ArrayList<Float> obstacleArray, float robotWidth, float ANGSTEPSIZE) {
		// Calculate the candidate directions based on obstacles
		// Threshold is to filter for noise
		// ANGSTEPSIZE: angle step size -> size of angles between readings 
		int thresh = 10;

		ArrayList<Float> canDir = new ArrayList<Float>(); 
		ArrayList<Float> canRanges= new ArrayList<Float>(); // Candidate ranges pairs, start, end
		
		// Simple edge detection, by threshold
		float prevVal = 0;
		float currentAngle = 0;
		for (float f : obstacleArray) {
			// Look for edges (to infinity)
			if ((Math.abs(f - prevVal) > thresh) && (f >= 200 || prevVal >= 200)) {
				canRanges.add(currentAngle);
			}
			currentAngle += ANGSTEPSIZE;
		}
		
		// Go through canRanges and assign candidate directions based on start and end points
		for (int i = 0; i < canRanges.size(); i += 2) {
			if (canRanges[2*i] - canRanges[2*i+1] > 2*robotWidth)
		}
		
	}

	public float getPCDCost(float c0) {
        // Use VFH*
		// Primary Candidate Direction
		// c_1 = m1 * Vcf(c_0, kt) + m2 * Vcf(c_0, 
		// Where kt = toGoal.heading/angRes
		
		float kt = myPose.angleTo(goal.getLocation())/angRes; // kt, absolute angle to goal
		float tn = myPose.getHeading(); // theta n, absolute angle current orientation
		
        System.out.println(String.format(">>> %f: kt = %f; tn = %f.\n", c0, kt, tn));

		return m[0] * DcCost(c0, kt) + m[1] * DcCost(c0, tn/angRes) + m[2] * DcCost(c0, mylastPose.getHeading());
	}
	
	public float DcCost(float c1, float c2) {
		// Delta C Cost
		// returns min{ |c1 - c2|, |c1 - c2 - 360/a|, |c1 - c2 + 360/a| }
		// where a is the angular resolution of the histogram
		
 		float firstmin = Math.min(Math.abs(c1-c2), Math.abs(c1-c2-FULLROT/angRes));
 		return Math.min(firstmin, Math.abs(c1-c2+FULLROT/angRes));
	}
}
