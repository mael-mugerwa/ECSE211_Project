
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class US {

  // robot constants
  public static int ROTATION_SPEED = 100;
  private double deltaTheta;
  private static final double TURNING_ERROR = 3.5 ;
     
  private Odometer odometer;
  private float[] usData;
  private EV3LargeRegulatedMotor leftMotor, rightMotor;
  private boolean Risingorfalling;
  private SampleProvider usDistance;

  //required ultrasonic constant
  private double d = 18.00;
  private double k = 2;

  /**
   * Constructor to initialize variables 
   * @param odo
   * @param leftMotor
   * @param rightMotor
   * @param localizationType
   * @param usDistance
   */
  public US(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			 SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}



	

	/**
	 * A method to localize position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < d + k) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (fetchUS() > d) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate out of the wall 
		while (fetchUS() < d + k) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > d) {
			leftMotor.forward();
			rightMotor.backward();
		}
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 
        //introduce a fix error correction
		leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, turningAngle-TURNING_ERROR), true);
		rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, turningAngle-TURNING_ERROR), false);

		// set odometer to theta = 0
		odometer.setTheta(0.0);

	}

	/**
	 * A method to get the distance from our sensor
	 * 
	 * @return
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * This method implement the conversion of a distance to rotation of each
	 * wheel need to cover the distance.
	 * 
	 * @param radius
	 * @param distance
	 * @return
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method implement the conversion of a angle to rotation of each
	 * wheel need to cover the distance.
	 * 
	 * @param radius
	 * @param distance
	 * @param angle
	 * @return
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}