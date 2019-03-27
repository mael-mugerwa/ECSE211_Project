import java.util.ArrayList;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class Navigation extends Thread{
	//Navigation related parameters
	final static double CM_ERR = 3;//distance error to target point
	//Optimal distance to scan a can
	final static int TARGET_DIST = 5;
	
	Odometer odometer;
	CanScanner canScanner;
	private static Navigation navigation = null; // Returned as singleton

	public EV3LargeRegulatedMotor leftMotor;
	public EV3LargeRegulatedMotor rightMotor;	
	public SampleProvider usDistance;
	public float[] usData; 	
	
	public boolean isNavigating = true;
	
	/**
	 * Navigation constructor
	 */
	public Navigation(Odometer odo, SampleProvider usDistance, CanScanner canScanner) {
		this.odometer = odo;
		this.canScanner = canScanner;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

	}

	
	//MAIN METHODS
	

	/**
	 * Makes the robot travel to the absolute field location (targetx, targety)
	 * This travelTo() uses obstacle detection and calls colorDetection when an object is detected 
	 * This method is used in our search function
	 *  
	 * @param targetx
	 * @param targety
	 * @return void
	 */
	public void travelTo(double targetx, double targety) {

		isNavigating = true;
		
		if (CanScanner.numberOfCansCollected <=3) {//robot is not full use travel to implementing canScanner
			double minAng = getAngle(targetx,targety);// get the angle
			this.turnTo(minAng);// turn to the minimal angle
			
			while (isNavigating) {
				int distance = getDistance();//distance reading from the usSensor
				
				if (distance < TARGET_DIST) { // if obstacle in the way
					setSpeeds(0,0);//stop robot
					canScanner.colorDetection();
				}

				double dx = targetx * Project.TILE - odometer.getX();
				double dy = targety * Project.TILE - odometer.getY();
				double travelDist = Math.sqrt(dx*dx+ dy*dy); //distance to travel to target point
				
				if (travelDist < CM_ERR) {// if robot is close enough to point
					isNavigating = false;//stop moving forward
				}
				
				// go forward
				setSpeeds(Project.FAST,Project.FAST);
			}
			
			//robot is not navigating stop it 
			setSpeeds(0,0);
		}
		
		else {//robot is full, travel back to origin
			travelBackToStartingCorner();
		}
		
	}
	
	/**
	 * Makes the robot travel to the absolute field location (targetx, targety)
	 * This travelTo() doesn't use obstacle detection  
	 *  
	 * @param targetx
	 * @param targety
	 * @return void
	 */
	public void travelTo2(double targetx, double targety) {

		isNavigating = true;
		double minAng = getAngle(targetx,targety);// get the angle
		this.turnTo(minAng);// turn to the minimal angle
			
		while (isNavigating) {
		
			double dx = targetx * Project.TILE - odometer.getX();
			double dy = targety * Project.TILE - odometer.getY();
			double travelDist = Math.sqrt(dx*dx+ dy*dy); //distance to travel to target point
				
			if (travelDist < CM_ERR) {// if robot is close enough to point
				isNavigating = false;//stop moving forward
			}
				
			// go forward
			setSpeeds(Project.FAST,Project.FAST);
		}
			
		//robot is not navigating stop it 
		setSpeeds(0,0);
	}
			
	/**
	 * Makes the robot travel to the starting corner
	 */
	public void travelBackToStartingCorner() {
		travelTo2(0,0);// travel to bridge exit
		travelTo2(0, 0);//travel to bridge entrance
		travelTo2(0, 0);//travel to starting corner
		canScanner.unloadingCans();
	}
	
	/**
	 * Makes the robot travel to the search area 
	 */
	public void travelToSearchArea() {
		travelTo2(0, 0);//travel to bridge entrance
		travelTo2(0, 0);//travel to bridge exit
		travelTo2(0, 0);//travel to search area
		//Localize;
	}
	
	/**
	 * Determines robot trajectory to search the area 
	 * @return 
	 */
	public ArrayList<double[]> searchTrajectory() {
		//Search Area 
	   	double[] start = {1,1};//starting corner
		double[] end = {4,4};//end corner
				
		int dx = (int) (end[0] - start[0]);// dx of the Search Area, used to determine how many points we need to travel to
		
		int numberofPoints = dx*2+1;//total number of points to travel to
				
		ArrayList<Double> Xwaypoints = new ArrayList<Double>();//all the X values for each point we need to travel to
				
		for(int i=0 ; i <= numberofPoints/2; i++) {//adds correct X value for each point we need to travel to
				Xwaypoints.add(start[0]+i);
				Xwaypoints.add(start[0]+i);
		}
		   
	    Xwaypoints.remove(0); //remove the first point which corresponds to the origin
			
///
			
		ArrayList<Double> Ywaypoints = new ArrayList<Double>();//all the Y values for each point we need to travel to

		for(int i=1 ; i<=numberofPoints; i=i+2) { //adds correct Y value for each point we need to travel to
			if(i%4 == 1 ) {
				Ywaypoints.add(end[1]);
				Ywaypoints.add(end[1]);
			}
			else {
				Ywaypoints.add(start[1]);
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
	 * This method calls the searchTrajectory method and makes the robot travel to each point 
	 */
	public void searchRoutine() {

		ArrayList<double[]> waypoints = searchTrajectory();
		
        for (double[] point : waypoints) { //for each point in double array waypoint
        	travelTo(point[0],point[1]);
        }
		
	}


	/**
	 * Causes the robot to turn (on point) to the absolute heading minAng
	 * calls getAngle()
	 * 
	 * @param angle
	 * @return void
	 */
	public void turnTo(double angle) {
		double minAng;
		
	    leftMotor.setSpeed(Project.SLOW);//set rotating speeds
	    rightMotor.setSpeed(Project.SLOW);
	    
		if (angle > 180) {//angle is not minimal angle
			minAng = 360.0-angle;// take opposite angle
			leftMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, minAng), true);//turn left at minimal angle instead of right
			rightMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, minAng), false);
		}
			
		else {//angle is minimal angle
			minAng = angle;
			leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, minAng), true);//turn right at minimal angle
			rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, minAng), false);	
		}		
	}
	

	/**
	 * Gets the angle the robot needs to turn at (right turn) to face the target destination
	 * 
	 * @param targetx
	 * @param targety
	 * @return double
	 */
	public double getAngle(double targetx , double targety) {

		double dx = targetx*Project.TILE - odometer.getX();
		double dy = targety*Project.TILE - odometer.getY();

		double angle = Math.atan2(dx, dy);
		double trajectoryAngle = angle*180/Math.PI-odometer.getTheta();
		
		if (trajectoryAngle <0)
			trajectoryAngle += 360.0;
		
		return trajectoryAngle;
	}
	
	/** 
	 * Returns true if another thread has called travelTo() or turnTo()
	 * and the method has yet to return; false otherwise
	 * 
	 * @return boolean
	 */
	private boolean isNavigating() {
		return isNavigating;
	}
	
	//HELPER METHODS
	
	/**
	 * Converting distance readings to an integer
	 * 
	 * @return integer
	 */
	public int getDistance() {
		usDistance.fetchSample(usData, 0); // acquire data
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
	   * This method allows the conversion of a distance to the total rotation of each wheel need to
	   * cover that distance.
	   * 
	   * @param radius
	   * @param distance
	   * @return integer
	   */
	 static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	 /**
	   * This method allows the conversion of a angle to the total rotation of each wheel 
	   * needed to turn at that angle 
	   * 
	   * @param radius
	   * @param width
	   * @param angle
	   * @return integer
	   */
	static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }


	public synchronized static Navigation getNavigation(Odometer odo, SampleProvider usSensorProvider, CanScanner canScanner) {
		if (navigation != null) { // Return existing object
		      return navigation;
		    } else { // create object and return it
		      navigation = new Navigation(odo, usSensorProvider, canScanner);
		      return navigation;
		    }
	}
	
}