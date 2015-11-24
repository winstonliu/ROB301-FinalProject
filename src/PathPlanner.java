import java.awt.geom.Point2D;
import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;

import lejos.robotics.geometry.Point;
import lejos.robotics.navigation.Pose;

public class PathPlanner {
	// Use VFH+

	// Goal orientation
	// If height of next step in the goal orientation direction exceeds height band, while not approaching goal,
	// add delta to orientation to keep within height band
	
	// Need timestep -> freq of refresh rate

public float FULLROT = 360; // depends on if degrees or radians

	// weight values for cost function
	public int[] m = {5, 2, 2};

	public PathPlanner() {
		// TODO Auto-generated constructor stub
	}
	
	public ArrayList<Float> calculateCandidateDir(float[] obstacleArray, float robotWidth, float ANGSTEPSIZE) {
		// Calculate the candidate directions based on obstacles
		// Threshold is to filter for noise
		// ANGSTEPSIZE: angle step size -> size of angles between readings 
		int thresh = 10;

		ArrayList<Float> canDir = new ArrayList<Float>(); 
		ArrayList<Point2D.Float> canRanges= new ArrayList<Point2D.Float>(); // Candidate ranges pairs, start, end
		
		// Simple edge detection, by threshold
		float prevVal = 0;
		float currentAngle = 0;
		for (float f : obstacleArray) {
			// Look for edges (to infinity)
			if (Math.abs(f - prevVal) > thresh && (f >= 200 || prevVal >= 200)){
                canRanges.add(new Point2D.Float(f,currentAngle)); // (height, angle)
			}
			prevVal = f;
			currentAngle += ANGSTEPSIZE;
		}
		
		// Go through canRanges and assign candidate directions based on start and end points
		int i = 0;
		while (i < canRanges.size()){
			// Check to make sure the gap is big enough
			if (canRanges.get(i).x < 200){ // starts with negative edge
				// ----        Top part is part we want
				//	   |____ 
				float gapTheta = canRanges.get(i).y - (float) (2*Math.asin((robotWidth/2)/canRanges.get(i).x));
				canDir.add(gapTheta);
				++i; // Increment to positive edge
			}
			// Case where there is a gap with two walls on either side
			else if (i+1 < canRanges.size()) {
				float delTheta = (canRanges.get(i+1).y - canRanges.get(i).y)/2;
                float avgDist = (canRanges.get(i+1).x + canRanges.get(i).x)/2;
                float halfGapSize = (float) (avgDist*Math.sin(delTheta));

                if (2*halfGapSize > robotWidth) { 
                    // Find candidate directions
                	canDir.add(canRanges.get(i).y + (float) (2*Math.asin((robotWidth/2)/canRanges.get(i).x)));
                	canDir.add(canRanges.get(i).y - (float) (2*Math.asin((robotWidth/2)/canRanges.get(i+1).x)));
                }
                i+=2; // This should make sure we only see positive edges from now on
			}
			// Case where there is an obstacle and nothing else on the other side
			else {
				float gapTheta = canRanges.get(i).y + (float) (2*Math.asin((robotWidth/2)/canRanges.get(i).x));
				canDir.add(gapTheta);
				++i;
			}
        }
		
		return canDir;
	}
	
	public float getOptimalDirection(ArrayList<Float> canDir, Pose myPose, Pose lastPose, Pose goal, float angRes) {
        // Coefficients
        // angRes = angular resolution of histogram
		float mincost = Float.POSITIVE_INFINITY;
		for (float f : canDir) {
			float cost = getPCDCost(f, myPose, lastPose, goal, angRes);
			if (cost < mincost) {
				mincost = cost;
			}
		}
		return mincost;
	}

	public float getPCDCost(float c0, Pose myPose, Pose mylastPose, Pose goal, float angRes) {
        // Use VFH*
		// Primary Candidate Direction
		// c_1 = m1 * Vcf(c_0, kt) + m2 * Vcf(c_0, 
		// Where kt = toGoal.heading/angRes
		
		float kt = myPose.angleTo(goal.getLocation())/angRes; // kt, absolute angle to goal
		float tn = myPose.getHeading(); // theta n, absolute angle current orientation
		
        System.out.println(String.format(">>> %f: kt = %f; tn = %f.\n", c0, kt, tn));

		return m[0] * DcCost(c0, kt, angRes) + m[1] * DcCost(c0, tn/angRes, angRes) + m[2] * DcCost(c0, mylastPose.getHeading(), angRes);
	}
	
	public float DcCost(float c1, float c2, float angRes) {
		// Delta C Cost
		// returns min{ |c1 - c2|, |c1 - c2 - 360/a|, |c1 - c2 + 360/a| }
		// where a is the angular resolution of the histogram
		
 		float firstmin = Math.min(Math.abs(c1-c2), Math.abs(c1-c2-FULLROT/angRes));
 		return Math.min(firstmin, Math.abs(c1-c2+FULLROT/angRes));
	}
}
