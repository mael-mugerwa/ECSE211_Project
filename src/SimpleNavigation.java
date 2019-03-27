import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.Sound;  

/**
 * @author Shaluo Wu
 * @author Glen Xu
 */
public class SimpleNavigation extends Thread {
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;


  /**
   * @param isNavigating a boolean variable used for keeping state of navigation
   * @param deltaX the x displacement required
   * @param deltaY the y displacement required
   * @param deltaTheta the angle difference required
   */
  private boolean isNavigating;
  private double deltaX;
  private double deltaY;
  private double theta;
  private double deltaTheta;
  private double distance;

  private Odometer odometer;

  public SimpleNavigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    try {
      this.odometer = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }

  /**
   * TravelTo method to let the robot travel to the expected location
   * 
   * @param x - x position that the robot should travel to
   * @param y - y coordinate that the robot should travel to
   */

  public void travelTo (double x, double y) {

    isNavigating = true; 

     
    deltaX = x - odometer.getX();
    deltaY = y - odometer.getY();
    distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

    if (deltaY == 0) {
        if (deltaX >= 0) {
          theta = 90;
        } 
        else {
          theta = -90;
        }
      }
      //calculate the theta that the robot should travel to 
      else {
        theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
      }    

    deltaTheta = theta - odometer.getTheta();
    
    if (deltaTheta > 180) { 
      deltaTheta -= 360;
    }
    else if (deltaTheta < -180) {
      deltaTheta += 360;
    }
    
    turnTo(deltaTheta);

    leftMotor.setSpeed(Project.FAST);
    rightMotor.setSpeed(Project.FAST);
    
    goStraight(distance); 
   
    odometer.setX(x);
    odometer.setY(y);
    odometer.setTheta(theta);
    
  isNavigating = false;

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

  /**
   * let the robot turn to deltaTheta
   * 
   */
  public void turnTo (double deltatheta) {

	  double theta1 = deltatheta; 
    leftMotor.setSpeed(Project.SLOW);
    rightMotor.setSpeed(Project.SLOW);

    //turn rightward if theta is greater than zero 
    if (theta1 > 0 && theta1 <= 180) {     
      turnRight(theta1); 
    }
    //turn leftward if theta is smaller than zero 
    else{    
      turnLeft(Math.abs(theta1));  
    }
  }
  
  /**
   * is Navigating method: return a boolean
   * 
   * @return isNavigating
   */
  public boolean isNavigating() {
    return isNavigating;
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
   * @param radius - the radius of the wheels
   * @param width - the distance between the two wheels
   * @param angle - the angle that the robot should turn
   * @return the convertDistance method
   */
  public int convertAngle(double angle) {
    return convertDistance(Math.PI * Project.TRACK * angle / 360.0);
  }
  
}


