import lejos.robotics.navigation.Pose;

public class PathPlanner {
	// Idea: using height map of obstacles, travel around the 
	// obstacles keeping above a certain height on the hill.
	
	// Using a method similar to Voroni, but using thresholds 	
	// Need:
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
