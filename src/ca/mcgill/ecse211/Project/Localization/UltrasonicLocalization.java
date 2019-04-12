package ca.mcgill.ecse211.Project.Localization;
import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Odometer.Odometer;
import ca.mcgill.ecse211.Project.Odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * In this class, we have our ultrasonic localization routine. We are using rising edge to orient
 * the robot correctly. Afterwards, the light localization routine can be performed
 * @author Pearl Wu
 *
 */
public class UltrasonicLocalization extends Thread {


  // initialize objects
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  private EV3UltrasonicSensor ultraSensor;
  private Odometer odometer;

  // initialize constants and variables
  private double alpha = 0.0;
  private double beta = 0.0;
  private double wallDetect = 60;
  private double fallingEdge = 50;
  private boolean leftWall = false;
  private boolean backWall = false;
  private final int FORWARD_SPEED = 150;
  private double distance;
  private double deltaTheta = 0;
  
  /**
   * create UltrasonicLocalization constructor
   * 
   * @param leftMotor
   * @param rightMotor
   * @param ultraSensor - helps to measure the distance between the robot and the wall
   */
  public UltrasonicLocalization(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      EV3UltrasonicSensor ultraSensor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    this.ultraSensor = ultraSensor;
    try {
      this.odometer = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }
  
  /**
   *  This is method is where we implementent the Ultrasonic Localization routine using rising edge
   */
  /**
   * FallingEdge: The case where the robot is supposed to be facing the wall
   */
  public void fallingEdge() {

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    SampleProvider provider = ultraSensor.getDistanceMode();
    float[] sample = new float[3];
    provider.fetchSample(sample, 0);

    // Adjust its orientation if robot is not facing the wall
    if (sample[0] * 100 <= wallDetect) {

      provider.fetchSample(sample, 0);

      // Turns CCW until it sees the wall
      while (sample[0] * 100 <= fallingEdge) {

        leftMotor.backward();
        rightMotor.forward();
        provider.fetchSample(sample, 0);

      }
    }

    // turn CCW if the robot did not detect a left wall
    while (leftWall == false) {

      leftMotor.backward();
      rightMotor.forward();

      provider.fetchSample(sample, 0);

      // if the robot meets a left wall, record the angle
      if (sample[0] * 100 < fallingEdge) {
        leftWall = true;
        alpha = odometer.getTheta();
      }
    }

    // turn right after the robot meets the leftwall
    leftMotor.forward();
    rightMotor.backward();

    // Pause for 3s, since the sensor updates fast
    try {
      Thread.sleep(3000);
    } catch (InterruptedException ex) {
    }

    // turn CW if the robot did not see a back wall
    while (backWall == false) {

      provider.fetchSample(sample, 0);

      //update the angle
      if (sample[0] * 100 < fallingEdge) {
        backWall = true;
        beta = odometer.getTheta();
      }
    }

    // update theta in odometer and turn the robot to zero degree
    odometer.setTheta(odometer.getTheta() + computeAngle(alpha, beta));
    turnTo(0.0);
  }
	/**
	 * This method gets Distance readings from the usSensor and casts them to int
	 * 
	 * @return integer
	 */
	public int getDistance() {
		SampleProvider provider = ultraSensor.getDistanceMode();
	    float[] sample = new float[provider.sampleSize()];
	    provider.fetchSample(sample, 0);
	    return (int)(sample[0]*100);
	}
	
  /**
   * @param alpha - angle obtained when detect the left wall
   * @param beta - angle obtained when detect the back wall
   * @return deltaTheta - the correct heading for the robot
   */
  public double computeAngle(double alpha, double beta) {
    double deltaTheta = 0;

    // calculate the angles
    if (alpha < beta) {
      deltaTheta = 45 - 0.5 * (alpha + beta);
    } else {
      deltaTheta = 225 - 0.5 * (alpha + beta);
    }
    return deltaTheta;
  }


  /**
   * let the robot turn to deltaTheta 
   */
  public void turnTo(double theta) {

    deltaTheta = theta - odometer.getTheta();

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    // deal with the maximum angle
    if (deltaTheta > 180) {
      deltaTheta = deltaTheta - 360;
    } else if (deltaTheta < -180) {
      deltaTheta = deltaTheta + 360;
    }

    // turn rightward if theta is greater than zero
    if (deltaTheta > 0 && deltaTheta <= 180) {
      leftMotor.rotate(convertAngle(deltaTheta), true);
      rightMotor.rotate(-convertAngle(deltaTheta), false);
    }
    // turn leftward if theta is smaller than zero
    else if (deltaTheta < 0 && deltaTheta >= -180) {
      deltaTheta = Math.abs(deltaTheta);
      leftMotor.rotate(-convertAngle(deltaTheta), true);
      rightMotor.rotate(convertAngle(deltaTheta), false);
    }
    leftMotor.stop();
    rightMotor.stop();

  }

  /**
   * convert distance method
   *
   * @param distance - the distance that the robot should travel
   * @return the distance that the wheels should travel
   */
  public int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * Project.WHEEL_RAD));
  }

  /**
   * convert angle method
   * 
   * @param angle - the angle that the robot should turn
   * @return the convertDistance method
   */
  public int convertAngle(double angle) {
    return convertDistance(Math.PI * Project.TRACK * angle / 360.0);
  }

  /**
   * Calculating the distance given x and y displacement
   * 
   * @param deltaX
   * @param deltaY
   */
  public double getDistance(double deltaX, double deltaY) {
    distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    return distance;
  }

  /**
   * Making the robot go straight for the distance entered
   * 
   * @param distance The distance required to move
   */
  public void goStraight(double distance) {
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }

  /**
   * Making the robot turn left for the angle entered
   * 
   * @param theta The angle required, to turn rightwards
   */
  public void turnLeft(double theta) {
    leftMotor.rotate(-convertAngle(theta), true);
    rightMotor.rotate(convertAngle(theta), false);
  }

  /**
   * Making the robot turn right for the angle entered
   * 
   * @param theta The angle required, to turn rightwards
   */
  public void turnRight(double theta) {
    leftMotor.rotate(convertAngle(theta), true);
    rightMotor.rotate(-convertAngle(theta), false);

  }


}
