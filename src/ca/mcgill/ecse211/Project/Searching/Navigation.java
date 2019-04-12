package ca.mcgill.ecse211.Project.Searching;
import java.util.ArrayList;

import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Localization.LightLocalization;
import ca.mcgill.ecse211.Project.Wifi.Wifi;
import ca.mcgill.ecse211.Project.Odometer.Odometer;
import ca.mcgill.ecse211.Project.Odometer.OdometerExceptions;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;  

/**
 * This class allows the robot to drive to a specified set of coordinates. It's methods are used everytime the robot travels as well as when searching. This was implemented using the Odometer
 * when an object is detected by the ultrasonic sensor while travelling, the robot calls the CanScanner methods to scan the can 
 * and figure out what should be done with it.
 * @author Mael Mugerwa
 * @author Shaluo Wu
 * @author Glen Xu
 */
public class Navigation extends Thread {
	
	/**
	 * Optimal distance to scan a can
	 */
	final static int TARGET_DIST = 3;
	/**
	 * x value for the center of the search zone
	 */
	public static double searchCenter_x;
	/**
	 * x value for the center of the search zone
	 */
	public static double searchCenter_y;
	
	private static Navigation nav = null; // Returned as singleton		
	
	/**
	 * boolean to know if can is picked up
	 */
	public static boolean picked = false;
  /**
   * @param isNavigating a boolean variable used for keeping state of navigation
   */
  private boolean isNavigating;
  /**
   * @param deltaX the x displacement required, used for traveling
   */
  private double deltaX;
  /**
   * @param deltaY the y displacement required, used for traveling
   */
  private double deltaY;
  /**
   * @param theta value used to calculate detlaTheta
   */
  private double theta;
  /**
   *@param deltaTheta the angle difference required, used for traveling
   */
  private double deltaTheta;
  /**
   * @param distance to travel to reach destination
   */
  private double distance;

  /**
   * odometer object
   */
  private Odometer odometer;
  /**
   * can scanner object
   */
  private CanScanner canScanner;
  
  /**
   * left motor object
   */
  private EV3LargeRegulatedMotor leftMotor;
  /**
   * right motor object
   */
  private EV3LargeRegulatedMotor rightMotor;
  /**
   * Us sensor sample provider used for searching
   */
  public SampleProvider usSensorProvider;
  /**
   * Array where us sensor readings are stored, used for searching
   */
  public float[] usData; 

  /**
   * Constructor
   * @param leftMotor
   * @param rightMotor
   * @param usSensorProvider
   * @param canScanner
   */
  public Navigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider usSensorProvider, CanScanner canScanner ) {
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    
	this.leftMotor.setAcceleration(3000);
	this.rightMotor.setAcceleration(3000);
    
    this.canScanner = canScanner;
	this.usSensorProvider = usSensorProvider;
	this.usData = new float[usSensorProvider.sampleSize()];
	
    try {
      this.odometer = Odometer.getOdometer();
    } catch (OdometerExceptions e) {
      e.printStackTrace();
    }
  }

  
  //SEARCH METHODS 
  /**
   * Makes the robot travel to the absolute field location (x, y)
   * This travelTo() uses obstacle detection and calls colorDetection when an object is detected 
   * This method is used in our search method
   * 
   * @param x - x position that the robot should travel to
   * @param y - y coordinate that the robot should travel to
   */
  public void SearchTravelTo (double x, double y) {

    isNavigating = true; 
    
	if (CanScanner.numberOfCansCollected < 2) {//robot is not full use travel to implementing canScanner
	
		deltaX = x - odometer.getX();
		deltaY = y - odometer.getY();
		distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));//calculate the distance separating current point from the target

		if (deltaY == 0) {
			if (deltaX >= 0) 
				theta = 90;
			else 
				theta = -90;		
		}
		    
		//calculate the theta that the robot should travel to 
		else 
			theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
	   
		deltaTheta = theta - odometer.getTheta();
		    
		if (deltaTheta > 180) 
			deltaTheta -= 360;
		else if (deltaTheta < -180) 
			deltaTheta += 360;
		    
		turnTo(deltaTheta);// turn to face destination

		leftMotor.setSpeed(Project.FORWARD_SPEED);
		rightMotor.setSpeed(Project.FORWARD_SPEED);
		    
		SearchGoStraight(distance, x , y ); // go straight calculated distance
	
		//else {
		odometer.setX(x);
		odometer.setY(y);
		odometer.setTheta(theta);
		//}
		
		isNavigating = false;
		
	}
	
	else {//robot is full, stop searching travel back to origin
		travelToStartCorner();
	}
  }

  /**
   * Makes the robot go straight for the distance entered
   * Waits for the move to be completed
   * 
   * @param distance The distance required to move
   */
  public void SearchGoStraight(double distance,double x ,double y) {
	leftMotor.setSpeed(Project.FORWARD_SPEED);
	rightMotor.setSpeed(Project.FORWARD_SPEED);

    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), true);
    
    while(leftMotor.isMoving() && rightMotor.isMoving()) {//while robot is traveling scan for a can in the way 		
    	if (getDistance() <= TARGET_DIST) { // if obstacle in the way
    		RegularGoStraight(5);
    		setSpeeds(0,0);//stop robot
			//for beta
			picked = canScanner.colorDetection();
			
			
			SearchTravelTo(x,y);//continue traveling to original destination
		}		
    }
    
  }
  
  /**
   * Makes the robot travel to the starting corner
   */
  public void travelToStartCorner() {
	  if(Wifi.startingCorner == 0) {
		  RegularTravelTo((Wifi.startZone_LL_x + 2)*30.48, (Wifi.startZone_LL_y + 2)*30.48);
		  canScanner.openDoor();
    	  RegularTravelTo((Wifi.startZone_LL_x)*30.48, (Wifi.startZone_LL_y)*30.48);
      }
      else if(Wifi.startingCorner == 1) {
    	  RegularTravelTo((Wifi.startZone_UR_x - 2)*30.48, (Wifi.startZone_LL_y + 2)*30.48);
    	  canScanner.openDoor();
    	  RegularTravelTo((Wifi.startZone_UR_x )*30.48, (Wifi.startZone_LL_y )*30.48);
      }
      else if(Wifi.startingCorner == 2) {
    	  RegularTravelTo((Wifi.startZone_UR_x - 2)*30.48, (Wifi.startZone_UR_y - 2)*30.48);
    	  canScanner.openDoor();
    	  RegularTravelTo((Wifi.startZone_UR_x)*30.48, (Wifi.startZone_UR_y)*30.48);
      }
      else if(Wifi.startingCorner == 3) {
    	  RegularTravelTo((Wifi.startZone_LL_x + 2)*30.48, (Wifi.startZone_UR_y - 2)*30.48);
    	  canScanner.openDoor();
    	  RegularTravelTo((Wifi.startZone_LL_x)*30.48, (Wifi.startZone_UR_y)*30.48);
      }
	  
	  RegularGoBack(30);
	  
	}
  
  /**
   * This method makes the robot turn to face it s destination
   * @param x
   * @param y
   */
  public void faceDestination(double x, double y) {
	  	deltaX = x - odometer.getX();
		deltaY = y - odometer.getY();
		distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));//calculate the distance separating current point from the target

		if (deltaY == 0) {
			if (deltaX >= 0) 
				theta = 90;
			else 
				theta = -90;		
		}
		    
		//calculate the theta that the robot should travel to 
		else 
			theta = Math.atan2(deltaX, deltaY) * 180 / Math.PI; 
	   
		deltaTheta = theta - odometer.getTheta();
		    
		if (deltaTheta > 180) 
			deltaTheta -= 360;
		else if (deltaTheta < -180) 
			deltaTheta += 360;
		    
		turnTo(deltaTheta);// turn to face destination
  }
  
  /**
   * This method makes the robot turn to face a certain theta
   */
  public void faceTheta(double theta) {
	  double LeftSide = theta + 180;// range for which minimal angle is to the left
	  if(LeftSide > 360) {
		  LeftSide =- 360; 		  
	  }
	  
 	  if(theta <= odometer.getTheta() && odometer.getTheta() <= LeftSide)// in most cases, will turn at minimal angle
 		  setSpeeds(-75,75);
 	  else
 		  setSpeeds(75,-75);
 	  
 	  while(true) {
		  if(Math.abs(odometer.getTheta() - theta)<= 1) {
			  setSpeeds(0,0);
			  break;
		  }
	  }
  }
	  
  /**
   * Makes the robot travel to the tunnel exit    
   */
  public void travelToTunnelExit(boolean applyCorrection) {
	  if(applyCorrection) {
		  faceDestination((Wifi.tunnelExit_x)*30.48, (Wifi.tunnelExit_y)*30.48);
		  nav.setSpeeds(150, 150);//go forward
			//booleans to know if a line was seen on the right or the left
			boolean right=false;
			boolean left=false;
					
			while(true) {
				if(LightLocalization.getRightRValue() < Project.BLACK) {	//grid line seen by right light sensor		
					rightMotor.setSpeed(0);// stop motor
					right=true;
				}
				if(LightLocalization.getLeftRValue() <Project.BLACK) { //grid line seen by left light sensor				
					leftMotor.setSpeed(0);//stop motor
					left=true;
				}
				if(left && right) {//both light sensors picked up the grid line
					leftMotor.setSpeed(150);
					rightMotor.setSpeed(150);
					//reverse to center of the tile
					RegularTravelTo((Wifi.tunnelExit_x)*30.48, (Wifi.tunnelExit_y)*30.48);			       
					left= false;
			       	right = false;
			       	break;
				}
			}
	  }
	  else
		  RegularTravelTo((Wifi.tunnelExit_x)*30.48, (Wifi.tunnelExit_y)*30.48);
  }
  
  /**
   * Makes the robot travel to the tunnel entrance 
   */
  public void travelToTunnelEntrance(boolean applyCorrection) {
	  if(applyCorrection) {
		  faceDestination((Wifi.tunnelEntrance_x)*30.48, (Wifi.tunnelEntrance_y)*30.48);
		  nav.setSpeeds(150, 150);//go forward
			//booleans to know if a line was seen on the right or the left
			boolean right=false;
			boolean left=false;
					
			while(true) {
				if(LightLocalization.getRightRValue() < Project.BLACK) {	//grid line seen by right light sensor		
					rightMotor.setSpeed(0);// stop motor
					right=true;
				}
				if(LightLocalization.getLeftRValue() <Project.BLACK) { //grid line seen by left light sensor				
					leftMotor.setSpeed(0);//stop motor
					left=true;
				}
				if(left && right) {//both light sensors picked up the grid line
					leftMotor.setSpeed(150);
					rightMotor.setSpeed(150);
					//reverse to center of the tile
					RegularTravelTo((Wifi.tunnelEntrance_x)*30.48, (Wifi.tunnelEntrance_y)*30.48);
			       	left= false;
			       	right = false;
			       	break;
				}
			}
	  }
	  else
		  RegularTravelTo((Wifi.tunnelEntrance_x)*30.48, (Wifi.tunnelEntrance_y)*30.48);
  }

	/**
	 * Our search method. It's assumed the robot is at the center of the search zone. It rotates on itself scanning for cans it is facing using the us sensor.
	 * Once a can is detected the robot travels forward until the us sensor detects it is in the optimal position to start scanning. The scanning and color detection methods from the canScanner class are called.
	 * The robot then reverses back to the search center to continue searching if the robot is not full.
	 */
	public void searchRoutine() {	
		if (CanScanner.numberOfCansCollected < 1) {//robot is not full use travel to implementing canScanner
			setSpeeds(-50,50);
			
			try {
				Thread.sleep(500);// to make sure a can is not detected multiple times in a row
			} catch (InterruptedException e) {
				//do nothing 
			}
			
			while(true) {
				if(getDistance() < 25) {
					
					setSpeeds(0,0);
					break;
				}
			}		
			
			setSpeeds(100,100);
			 while(leftMotor.isMoving() && rightMotor.isMoving()) {//while robot is traveling scan for a can in the way 		
			    	if (getDistance() <= 3) { // if obstacle in the way
			    		RegularGoStraight(5);
			    		setSpeeds(0,0);//stop robot
						picked = canScanner.colorDetection();
						reverseBacktoSearchCenter();
						searchRoutine();
			    	}
			 }
		}
		
	}
	
	/**
	 * This method makes the robot travel to the center of the search zone
	 */
	public void travelToSearchCenter() {
		double dx = (Wifi.searchZone_UR_x - Wifi.searchZone_LL_x)/2;
		double dy = (Wifi.searchZone_UR_y - Wifi.searchZone_LL_y)/2 ;
		searchCenter_x = (Wifi.searchZone_LL_x + dx)*30.48;
		searchCenter_y = (Wifi.searchZone_LL_y + dy)*30.48;
		
		SearchTravelTo( searchCenter_x , searchCenter_y );
	}
	
	/**
	 * This method makes the robot reverse back to the center of the search zone by having the robot travel backwards the distance separating it from that point
	 * This method is used during our search routine
	 */
	public void reverseBacktoSearchCenter() {		
		//distance from where the robot is to search center
		double deltax = searchCenter_x - odometer.getX();
		double deltay = searchCenter_y - odometer.getY();
		distance = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));//calculate the distance separating current point from the search center
		RegularGoBack(distance);
	}
	
	
  //REGULAR NAVIGATION METHODS
  
  /**
   * Makes the robot travel to the absolute field location (x, y)
   * This travelTo() doesn't use obstacle detection 
   * 
   * @param x - x position that the robot should travel to
   * @param y - y coordinate that the robot should travel to
   */
  public void RegularTravelTo (double x, double y) {

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

    leftMotor.setSpeed(Project.FORWARD_SPEED);
    rightMotor.setSpeed(Project.FORWARD_SPEED);
    
    RegularGoStraight(distance); 
   
    odometer.setX(x);
    odometer.setY(y);
    odometer.setTheta(theta);
    
  isNavigating = false;

  }

  /**
   * Makes the robot go straight for the distance entered
   * Doesn't wait for the move to be completed
   * 
   * @param distance The distance required to move
   */
  public void RegularGoStraight(double distance) {
	  
	  leftMotor.setSpeed(150);
	  rightMotor.setSpeed(150);
	  leftMotor.setAcceleration(3000);
    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), false);
  }
  
  /**
   * Makes the robot go straight for the distance entered
   * Doesn't wait for the move to be completed
   * @param distance The distance required to move
   */
public void RegularGoBack(double distance) {
	  
	  leftMotor.setSpeed(100);
	  rightMotor.setSpeed(100);
	  leftMotor.setAcceleration(3000);
    leftMotor.rotate(-convertDistance(distance), true);
    rightMotor.rotate(-convertDistance(distance), false);
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
   * Makes the robot turn to deltaTheta
   * @param deltaTheta
   */
  public void turnTo (double deltaTheta) {

	  double theta1 = deltaTheta; 
    leftMotor.setSpeed(Project.ROTATE_SPEED);
    rightMotor.setSpeed(Project.ROTATE_SPEED);

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
   * This method allows the conversion of a distance to the total rotation of each wheel need to
   * cover that distance.
   * 
   * @param distance
   * @return
   */
  public static int convertDistance(double distance) {
    return (int) ((180.0 * distance) / (Math.PI * Project.WHEEL_RAD));
  }

  /**
   * This method allows the conversion of a angle to the total rotation of each wheel 
   * needed to turn at that angle 
   * 
   * @param angle
   * @return
   */
  public int convertAngle(double angle) {
    return convertDistance(Math.PI * Project.TRACK * angle / 360.0);
  }
  
	/**
	 * Converting distance readings to an integer
	 * 
	 * @return integer
	 */
	public int getDistance() {
		usSensorProvider.fetchSample(usData, 0); // acquire data
		return (int) (usData[0] * 100); // extract from buffer, cast to int
	}
  
	/**
	 * sets both wheel speeds and makes the robot either move forward or backwards based on those speeds
	 * 
	 * @param lSpd
	 * @param rSpd
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	/**
	 * get current instance of navigation
	 * @param leftMotor
	 * @param rightMotor
	 * @param usSensorProvider
	 * @param canScanner
	 * @return
	 */
	public synchronized static Navigation getNavigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider usSensorProvider, CanScanner canScanner) {
		if (nav != null) { // Return existing object
		      return nav;
		    } else { // create object and return it
		      nav = new Navigation(leftMotor, rightMotor, usSensorProvider, canScanner);
		      return nav;
		    }
	}
	
}


