import java.awt.Point;

import javax.print.attribute.standard.Destination;

import org.freedesktop.DBus.Description;
import org.omg.CORBA.PUBLIC_MEMBER;

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
	public Boolean finalApproach;
	
	public pathplanner(Pose currentPose, lejos.robotics.geometry.Point destination) {
		// TODO Auto-generated constructor stub
		myPose = currentPose;
		goal = destination;
		finalApproach = Boolean.FALSE;
	}
	
	public lejos.robotics.geometry.Point straightPathOrientation () {
		return goal.subtractWith(myPose.getLocation());
	}
	
	public void planPath() {
		lejos.robotics.geometry.Point straightPath = straightPathOrientation();
		
		// Find closest candidate valley, gradient descent?
	}

}
