import lejos.hardware.motor.Motor;

// Motor D

public class ClawControl {
	public ClawControl() {
		// TODO Auto-generated constructor stub
		// Default speed 100
		Motor.D.setSpeed(180);
		Motor.D.rotateTo(0);
	}
	
	public ClawControl(int speed){
        Motor.D.setSpeed(speed);
		Motor.D.rotateTo(0);
	}
	
	public void openClaw(){ Motor.D.rotateTo(-180); }
	public void resetToZero(){ Motor.D.rotateTo(0); }
	public void liftClaw(){ Motor.D.rotateTo(270); }
}
