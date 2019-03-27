import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;


public class UltrasonicLocalizer extends Thread {


  // initialize objects
  private static EV3LargeRegulatedMotor leftMotor;
  private static EV3LargeRegulatedMotor rightMotor;
  private static EV3UltrasonicSensor ultraSensor;
  private static Odometer odometer;

  // initialize constants and variables
  private static double alpha = 0.0;
  private static double beta = 0.0;
  private static double wallDetect = 60;
  private static double fallingEdge = 50;
  private static double risingEdge = 50;
  private static boolean leftWall = false;
  private static boolean backWall = false;
  private static final int FORWARD_SPEED = 150;
  private static double distance;
  private static double deltaTheta = 0;
  
  /**
   * create UltrasonicLocalization constructor
   * 
   * @param leftMotor
   * @param rightMotor
   * @param ultraSensor - helps to measure the distance between the robot and the wall
   */
  public UltrasonicLocalizer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
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
   *  FallingEdge: The case where the robot is not supposed to be facing the wall
   */
  public void risingEdge() {

    leftMotor.setSpeed(FORWARD_SPEED);
    rightMotor.setSpeed(FORWARD_SPEED);

    SampleProvider provider = ultraSensor.getDistanceMode();
    float[] sample = new float[3];
    provider.fetchSample(sample, 0);

    // Turns CCW until it sees the wall
    if (sample[0] * 100 >= wallDetect) {

      provider.fetchSample(sample, 0);

      while (sample[0] * 100 > risingEdge) {
        leftMotor.backward();
        rightMotor.forward();
        provider.fetchSample(sample, 0);
      }
    }

    // turn CCW if the robot did not see a back wall
    while (backWall == false) {

      leftMotor.backward();
      rightMotor.forward();

      provider.fetchSample(sample, 0);

      //update the angle if the robot sees the wall
      if (sample[0] * 100 > risingEdge) {
        backWall = true;
        beta = odometer.getTheta();
        System.out.println("detected 1st wall");
      }
    }

    // turn right after detecting the backwall
    leftMotor.forward();
    rightMotor.backward();

    // Pause for 3s, since the sensor updates fast
    try {
      Thread.sleep(2000);
    } catch (InterruptedException ex) {
    }

    // continue to turn right if the robot did not detect the left wall
    while (leftWall == false) {
      provider.fetchSample(sample, 0);

      if (sample[0] * 100 > risingEdge) {
        leftWall = true;
        alpha = odometer.getTheta();
        System.out.println("detected 2nd wall");

      }
    }

    odometer.setTheta(odometer.getTheta() + computeAngle(alpha, beta));

    turnTo(-10);
    
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
 * This method calculates the mean of color reading array
 * @param colorArray
 * @return
 */
	  public float getRegMean(float[] colorArray) {
	  	  float sum=0;
	        for (float val : colorArray) 
	        	sum += val;
	        return sum/colorArray.length;
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
   * 
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
