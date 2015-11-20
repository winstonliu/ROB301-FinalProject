import java.util.ArrayList;
import lejos.robotics.navigation.Pose;

public class TestPath {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Pose currentPose = new Pose(5, 7, 10);
		Pose lastPose = new Pose(5, 4, 20);
		Pose goalPose = new Pose(5, 6, 0);
		PathPlanner myPlanner = new PathPlanner(currentPose,lastPose, goalPose);
		float[] obstacleArray = {10, 60, 180};
		ArrayList<Float> AngleCost = new ArrayList<Float>();

		for (float f : obstacleArray) {
			float t = myPlanner.getPCDCost(f);
			AngleCost.add(t);
			System.out.println(String.format("Cost of %f is %f.\n", f, t));
		}
	}

}
