package ca.mcgill.ecse211.Project.Searching;
import java.util.ArrayList;

import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Localization.LightLocalization;
import ca.mcgill.ecse211.Project.Wifi.Wifi;
import ca.mcgill.esce211.Project.Odometer.Odometer;
import ca.mcgill.esce211.Project.Odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;  

/**
 * This class allows the robot to drive to a specified set of coordinates. This was implemented using the Odometer
 * when an object is detected by the ultrasonic sensor while travelling, the robot calls the CanScanner methods to scan the can 
 * and figure out what should be done with it.
 * @author Mael Mugerwa
 * @author Shaluo Wu
 * @author Glen Xu
 */
public class Navigation extends Thread {
	
	//Optimal distance to scan a can
	final static int TARGET_DIST = 3;
	public static double searchCenter_x;
	public static double searchCenter_y;
	
	private static Navigation nav = null; // Returned as singleton		
	
	//Only for beta
	public static boolean picked = false;
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
  private CanScanner canScanner;
  
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;
  public SampleProvider usSensorProvider;
  public float[] usData; 

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

		leftMotor.setSpeed(Project.FAST);
		rightMotor.setSpeed(Project.FAST);
		    
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
	leftMotor.setSpeed(Project.FAST);
	rightMotor.setSpeed(Project.FAST);

    leftMotor.rotate(convertDistance(distance), true);
    rightMotor.rotate(convertDistance(distance), true);
    
    while(leftMotor.isMoving() && rightMotor.isMoving()) {//while robot is traveling scan for a can in the way 		
    	if (getDistance() <= TARGET_DIST) { // if obstacle in the way
			setSpeeds(0,0);//stop robot
			//for beta
			picked = canScanner.colorDetection();
			if(picked) {
				Sound.setVolume(100);
				for(int i=0; i<5; i++) 
					Sound.beep();				
			}
			
			SearchTravelTo(x,y);//continue traveling to original destination
		}		
    }
    
  }
  
  /**
   * Makes the robot travel to the starting corner
   */
  public void travelToStartCorner() {
	  if(Wifi.startingCorner == 0) {
		  System.out.println("travel to "+(Wifi.startZone_LL_x + 0.5)+" "+(Wifi.startZone_LL_y + 0.5));
    	  RegularTravelTo((Wifi.startZone_LL_x + 0.5)*30.48, (Wifi.startZone_LL_y + 0.5)*30.48);
      }
      else if(Wifi.startingCorner == 1) {
    	  RegularTravelTo((Wifi.startZone_UR_x - 0.5)*30.48, (Wifi.startZone_LL_y + 0.5)*30.48);
      }
      else if(Wifi.startingCorner == 2) {
    	  RegularTravelTo((Wifi.startZone_UR_x - 0.5)*30.48, (Wifi.startZone_UR_y - 0.5)*30.48);
      }
      else if(Wifi.startingCorner == 3) {
    	  RegularTravelTo((Wifi.startZone_LL_x + 0.5)*30.48, (Wifi.startZone_UR_y - 0.5)*30.48);
      }
	  
	  canScanner.unloadingCans();//unload cans
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
		  nav.setSpeeds(75, 75);//go forward
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
					leftMotor.setSpeed(75);
					rightMotor.setSpeed(75);
					//reverse to center of the tile
					RegularTravelTo((Wifi.tunnelExit_x)*30.48, (Wifi.tunnelExit_y)*30.48);			       	left= false;
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
		  nav.setSpeeds(75, 75);//go forward
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
					leftMotor.setSpeed(75);
					rightMotor.setSpeed(75);
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
	 * Determines robot trajectory to search the area 
	 * Returns each point to travelTo to cover the entire search area
	 * 
	 * @return 
	 */
	public ArrayList<double[]> searchTrajectory() {
		//Search Area 
	   	double[] start = {Wifi.searchZone_LL_x, Wifi.searchZone_LL_x};//starting corner
		double[] end = {Wifi.searchZone_UR_x,Wifi.searchZone_UR_y};//end corner
		
		int dx = (int) (end[0] - start[0]);// dx of the Search Area, used to determine how many points we need to travel to
		
		int numberofPoints = dx*2+1;//total number of points to travel to
				
		ArrayList<Double> Xwaypoints = new ArrayList<Double>();//all the X values for each point we need to travel to
				
		for(int i=0 ; i <= numberofPoints/2; i++) {//adds correct X value for each point we need to travel to
				Xwaypoints.add(start[0]+i);
				Xwaypoints.add(start[0]+i);
		}
		   
	    Xwaypoints.remove(0); //remove the first point which corresponds to the origin
			
		ArrayList<Double> Ywaypoints = new ArrayList<Double>();//all the Y values for each point we need to travel to

		for(int i=1 ; i<=numberofPoints; i=i+2) { //adds correct Y value for each point we need to travel to
			if(i%4 == 1 ) {
				Ywaypoints.add(end[1]);//add Search_UR_Y
				Ywaypoints.add(end[1]);
			}
			else {
				Ywaypoints.add(start[1]);//add Search_LL_Y
				Ywaypoints.add(start[1]);
			}			
		}
		
		Ywaypoints.remove(Ywaypoints.size()-1);//remove the last point which corresponds to a point of the search area
			
///
			
		ArrayList<double[]> waypoints = new ArrayList<double[]>();//ArrayList holding all the waypoints we need to travel to 
		for (int j = 0; j < Xwaypoints.size() ; j++) {
			double[] point = {Xwaypoints.get(j),Ywaypoints.get(j)};//setting each point appropriately
			waypoints.add(point);
		}
	    	
		return waypoints;
	}
	
	/**
	 * This is our search method.
	 * It calls the searchTrajectory method and makes the robot travel to each point of the search area
	 */
	public void searchRoutine() {

		ArrayList<double[]> waypoints = searchTrajectory();
		
      for (double[] point : waypoints) { //for each point in double array waypoint
      	SearchTravelTo(point[0]*Project.TILE,point[1]*Project.TILE);
      }		
	}
	
	public void searchRoutine2() {	
		if (CanScanner.numberOfCansCollected < 2) {//robot is not full use travel to implementing canScanner
			setSpeeds(-50,50);
			
			try {
				Thread.sleep(500);// to make sure a can is not detected multiple times in a row
			} catch (InterruptedException e) {
				//do nothing 
			}
			
			while(true) {
				//System.out.println(getDistance());
				if(getDistance() < 25) {
					try {
						Thread.sleep(1000);// to make sure the robot is perfectly facing the can
					} catch (InterruptedException e) {
						//do nothing 
					}
					
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
						searchRoutine2();
			    	}
			 }
		}
		
	}
	
	public void travelToSearchCenter() {
		double dx = (Wifi.searchZone_UR_x - Wifi.searchZone_LL_x)/2;
		double dy = (Wifi.searchZone_UR_y - Wifi.searchZone_LL_y)/2 ;
		searchCenter_x = (Wifi.searchZone_LL_x + dx)*30.48;
		searchCenter_y = (Wifi.searchZone_LL_y + dy)*30.48;
		
		SearchTravelTo( searchCenter_x , searchCenter_y );
	}
	
	public void reverseBacktoSearchCenter() {		
		//distance from where the robot is to search center
		double deltax = searchCenter_x - odometer.getX();
		double deltay = searchCenter_y - odometer.getY();
		distance = Math.sqrt(Math.pow(deltax, 2) + Math.pow(deltay, 2));//calculate the distance separating current point from the search center
		System.out.println(distance);
		RegularGoBack(distance);
	}
	
	public void test() {
		setSpeeds(-50,50);
		while(true) {
			System.out.println(getDistance());
			if(getDistance() < 25) {
				Sound.beep();
				
			}
		}		
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

    leftMotor.setSpeed(Project.FAST);
    rightMotor.setSpeed(Project.FAST);
    
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
	  
	  leftMotor.setSpeed(100);
	  rightMotor.setSpeed(100);
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
   * @param deltatheta
   */
  public void turnTo (double deltaTheta) {

	  double theta1 = deltaTheta; 
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

	
	public synchronized static Navigation getNavigation(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, SampleProvider usSensorProvider, CanScanner canScanner) {
		if (nav != null) { // Return existing object
		      return nav;
		    } else { // create object and return it
		      nav = new Navigation(leftMotor, rightMotor, usSensorProvider, canScanner);
		      return nav;
		    }
	}
	
}


