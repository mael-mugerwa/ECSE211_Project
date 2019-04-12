package ca.mcgill.ecse211.Project.Localization;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Odometer.Odometer;
import ca.mcgill.ecse211.Project.Odometer.OdometerExceptions;
import ca.mcgill.ecse211.Project.Searching.Navigation;
import ca.mcgill.ecse211.Project.Wifi.Wifi;

/**
 * This class houses all the required methods for our light localization routine
 * 
 * @author Mael
 *
 */
public class LightLocalization  {
	
	/**
	 * US distance to reach for the can to be in an optimal position to be scanned 
	 */
	final static int TARGET_DIST = 3;
	/**
	 * Distance to reverse before picking up can
	 */
	final static int BACKWARD = 10;
	/**
	 * Distance need to go forward to pick up can
	 */
	final static int FORWARD = 13;

	/**
	 * left motor object
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * right motor object
	 */
	private EV3LargeRegulatedMotor rightMotor;	
	
	/**
	 * left light sensor provider object
	 */
	private static SampleProvider leftLightProvider;
	/**
	 * array where left light sensor readings are stores
	 */
	private static float[] leftLightData;
	
	/**
	 * right light sensor provider object
	 */
	private static SampleProvider rightLightProvider;
	/**
	 * array where right light sensor readings are stores
	 */
	private static float[] rightLightData;
	
	/**
	 * odometer object
	 */
	Odometer odo;
	/**
	 * navigation object
	 */
	Navigation nav;
	
	/**
	 * Constructor 
	 * @param leftMotor
	 * @param rightMotor
	 * @param usSensorProvider
	 * @param leftLightProvider
	 * @param rightLightProvider
	 */
	public LightLocalization (EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			SampleProvider usSensorProvider,
			SampleProvider leftLightProvider,
			SampleProvider rightLightProvider,
			Navigation navigation) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		

		this.leftMotor.setAcceleration(3000);
		this.rightMotor.setAcceleration(3000);
		
		LightLocalization.leftLightProvider = leftLightProvider;
		leftLightData = new float[3];	
			
		LightLocalization.rightLightProvider = rightLightProvider;
		rightLightData = new float[3];	
		
		this.nav = navigation;
		
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}
	
	/**
	 * This is our new light localization routine, the robot goes forward until it sees a black line, aligns itself
	 * turns 90 degrees in the correct correction to face the closest grid point,
	 * goes forward until sees a black line and aligns itself on the point
	 */
	public void localize() {
		nav.setSpeeds(75,75);//go forward
		
		//booleans to know if a line was seen on the right or the left
		boolean right=false;
		boolean left=false;
		
		while(true) {
			if(getRightRValue() < Project.BLACK) {	//grid line seen by right light sensor		
				rightMotor.setSpeed(0);// stop motor
				right=true;
			}
			if(getLeftRValue() <Project.BLACK) { //grid line seen by left light sensor				
				leftMotor.setSpeed(0);//stop motor
				left=true;
			}
			if(left && right) {//both light sensors picked up the grid line
				leftMotor.setSpeed(75);
				rightMotor.setSpeed(75);
				//reverse distance separating lights sensors from wheels 
				leftMotor.rotate(Navigation.convertDistance(Project.DIST), true);
	      		rightMotor.rotate(Navigation.convertDistance(Project.DIST), false);
	        	nav.setSpeeds(0, 0);
	        	left= false;
	        	right = false;
	        	break;
			}
		}
		
		
			leftMotor.setSpeed(75);
			rightMotor.setSpeed(75);
			leftMotor.rotate(nav.convertAngle(90), true);
			rightMotor.rotate(-nav.convertAngle(90), false);	
					
			nav.setSpeeds(75,75);// go forward
			right=false;
			left=false;
			
			// repeat same logic as above
			while(true) {
				if(getRightRValue() < Project.BLACK) {					
					rightMotor.setSpeed(0);
					right=true;
				}
				if(getLeftRValue() <Project.BLACK) {					
					leftMotor.setSpeed(0);
					left=true;
				}
				if(left && right) {
					leftMotor.setSpeed(75);
					rightMotor.setSpeed(75);
					leftMotor.rotate(Navigation.convertDistance(Project.DIST), true);
		      		rightMotor.rotate(Navigation.convertDistance(Project.DIST), false);
		        	nav.setSpeeds(0, 0);
		        	left= false;
		        	right = false;
		        	break;
				}
			}
		update();//update the odometer based on starting corner
	}
	
	/**
	 * This method is used to relocalize only on one axis depending on the tunnel orientation.
	 * It corrects by re aligning the robot on the opposite axis of the tunnel's orientation
	 * @return
	 */
	public double reLocalize() {
		double ang;
		if(Wifi.tunnelIsHorizontal) {
			nav.faceTheta(0);
			ang = 0;
		}
		else {
			nav.faceTheta(90);
			ang = 90;
		}
		
		nav.setSpeeds(75, 75);//go forward
		//booleans to know if a line was seen on the right or the left
		boolean right=false;
		boolean left=false;
				
		while(true) {
			if(getRightRValue() < Project.BLACK) {	//grid line seen by right light sensor		
				rightMotor.setSpeed(0);// stop motor
				right=true;
			}
			if(getLeftRValue() <Project.BLACK) { //grid line seen by left light sensor				
				leftMotor.setSpeed(0);//stop motor
				left=true;
			}
			if(left && right) {//both light sensors picked up the grid line
				leftMotor.setSpeed(75);
				rightMotor.setSpeed(75);
				//reverse to center of the tile
				nav.RegularGoBack(Project.TILE /2 - Project.DIST);
		       	nav.setSpeeds(0, 0);
		       	left= false;
		       	right = false;
		       	break;
			}
		}		
		return ang;
	}
	/**
	 * This method is used to relocalize on both axis by facing the tunnel entrance, reversing and correcting on that axis then turning 90 degrees and doing the same to correct on the other axis.
	 * @return
	 */
public double reLocalizeAgain() {
		double ang; 
		
		nav.faceDestination(Wifi.tunnelEntrance_x * 30.48, Wifi.tunnelEntrance_y*30.48);
			
		nav.setSpeeds(-75, -75);//reverse
		//booleans to know if a line was seen on the right or the left
		boolean right=false;
		boolean left=false;
				
		while(true) {
			if(getRightRValue() < Project.BLACK) {	//grid line seen by right light sensor		
				rightMotor.setSpeed(0);// stop motor
				right=true;
			}
			if(getLeftRValue() <Project.BLACK) { //grid line seen by left light sensor				
				leftMotor.setSpeed(0);//stop motor
				left=true;
			}
			if(left && right) {//both light sensors picked up the grid line
				leftMotor.setSpeed(75);
				rightMotor.setSpeed(75);
				//reverse to center of the tile
				nav.RegularGoStraight(Project.DIST);
		       	nav.setSpeeds(0, 0);
		       	left= false;
		       	right = false;
		       	break;
			}
		}		
		
		if(Wifi.tunnelIsHorizontal) 
			nav.faceTheta(0);


		else 
			nav.faceTheta(90);
		
		ang = reLocalize();
		return ang;
	}
	
	/**
	 * This method returns the R Color Reading from the right light sensor
	 * @return
	 */
	public static float getRightRValue() {
		rightLightProvider.fetchSample(rightLightData, 0); // acquire data
		return rightLightData[0];
  }
	/**
	 * 
	 * This method returns the R Color Reading from the left light sensor
	 * @return
	 */	 
	public static float getLeftRValue() {
		leftLightProvider.fetchSample(leftLightData, 0); // acquire data
		return leftLightData[0];
  }
	
	/**
	 * This method sets the odometer values correctly based on starting corner. This method is used after the main localize routine
	 */
	public void update() {
		if(Wifi.startingCorner == 0) 
			odo.setXYT(1*Project.TILE, 1*Project.TILE, 90);
		
		else if (Wifi.startingCorner == 1) 
			odo.setXYT(14*Project.TILE, 1*Project.TILE, 0);
			//odo.setXYT(7*Project.TILE, 1*Project.TILE, 0);
		
		else if (Wifi.startingCorner == 2) 
			odo.setXYT(14*Project.TILE, 8*Project.TILE, 270);
			//odo.setXYT(14*Project.TILE, 8*Project.TILE, 270);
		
		else if (Wifi.startingCorner == 3) 
			odo.setXYT(1*Project.TILE, 8*Project.TILE, 180);		
	}
	
	
}

