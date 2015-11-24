import java.util.ArrayList;
import lejos.robotics.navigation.Pose;

public class TestPath {

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		Pose currentPose = new Pose(5, 7, 10);
		Pose lastPose = new Pose(5, 4, 20);
		Pose goalPose = new Pose(5, 6, 0);
		PathPlanner myPlanner = new PathPlanner();

		ArrayList<Float> AngleCost  = myPlanner.calculateCandidateDir(obstacleArray, (float) (10), (float)(5));
		float dir = myPlanner.getOptimalDirection(AngleCost, currentPose,lastPose, goalPose, (float) (5.0));

        System.out.println(String.format("Dir is %f.\n", dir));
	}

}
