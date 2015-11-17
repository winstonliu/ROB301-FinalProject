import java.awt.List;
import java.awt.Point;
import java.io.FilterInputStream;
import java.util.ArrayList;

import javax.print.attribute.standard.Destination;
import javax.security.auth.kerberos.KerberosTicket;

import org.freedesktop.DBus.Description;
import org.omg.CORBA.PUBLIC_MEMBER;
import org.omg.IOP.CodeSets;

import lejos.robotics.navigation.Pose;

public class pathplanner {

	// Idea: using height map of obstacles, travel around the 
	// obstacles keeping above a certain height on the hill.
	
	// Using a method similar to Voroni, but using thresholds 	
	// Need:
	// Goal orientation
	// If height of next step in the goal orientation direction exceeds height band, while not approaching goal,
	// add delta to orientation to keep within height band
	
	// Need timestep -> freq of refresh rate

	public lejos.robotics.geometry.Point goal;
	public Pose myPose;
	static Pose lastPose;
	public Boolean finalApproach;
	public lejos.robotics.geometry.Point toGoal;
	public float projDist; // dt
	
	// Coefficients
	public float angRes; // angular resolution of histogram
	public float FULLROT = 360; // depends on if degrees or radians
	// weight values for cost function
	public int m1; // mu1
	public int m2; // mu2
	public int m3; // mu3
	
	public pathplanner(Pose currentPose, lejos.robotics.geometry.Point destination) {
		// TODO Auto-generated constructor stub
		myPose = currentPose;
		goal = destination;
		toGoal = goal.subtractWith(myPose.getLocation());
		finalApproach = Boolean.FALSE;
	}

	public float getPCDCost(float c0) {
        // Use VFH*
		// Primary Candidate Direction
		// c_1 = m1 * Vcf(c_0, kt) + m2 * Vcf(c_0, 
		// Where kt = toGoal.heading/angRes
		
		float kt = toGoal.angle()/angRes; // kt
		float tn = myPose.getHeading(); // theta n, current orientation

		return m1 * DcCost(c0, kt) + m2 * DcCost(c0, tn/angRes) + m3 * DcCost(c0, lastPose.getHeading());
	}
	
	public float DcCost(float c1, float c2) {
		// Delta C Cost
		// returns min{ |c1 - c2|, |c1 - c2 - 360/a|, |c1 - c2 + 360/a| }
		// where a is the angular resolution of the histogram
		
 		float firstmin = Math.min(Math.abs(c1-c2), Math.abs(c1-c2-FULLROT/angRes));
 		return Math.min(firstmin, Math.abs(c1-c2+FULLROT/angRes));
	}
}
