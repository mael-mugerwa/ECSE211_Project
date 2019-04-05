import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.ArrayList;

import lejos.hardware.Sound;

public class FullLocalization  {
	
	private static FullLocalization canScanner = null; // Returned as singleton
	
	//Optimal distance to scan a can
	final static int TARGET_DIST = 3;
	//Distance to reverse before picking up can
	final static int BACKWARD = 10;
	//Distance need to go forward to pick up can
	final static int FORWARD = 13;
	
	public static final double TILE = 30.48;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;	
	private EV3MediumRegulatedMotor arm;
	private EV3LargeRegulatedMotor gate;
	
	private SampleProvider leftLightProvider;
	private float[] leftLightData;
	
	private SampleProvider rightLightProvider;
	private float[] rightLightData;
	
	private SampleProvider usProvider;
	private float[] usData;	
	
	public final double BLACK =  Project.BLACK;
	Odometer odo;
	
	public FullLocalization (EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			SampleProvider usSensorProvider,
			SampleProvider leftLightProvider,
			SampleProvider rightLightProvider) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		

		this.leftMotor.setAcceleration(3000);
		this.rightMotor.setAcceleration(3000);
		
		this.leftLightProvider = leftLightProvider;
		this.leftLightData = new float[3];	
			
		this.rightLightProvider = rightLightProvider;
		this.rightLightData = new float[3];	
			
		this.usProvider = usSensorProvider;
		this.usData = new float[usSensorProvider.sampleSize()];
  	  
		try {
			this.odo = Odometer.getOdometer();
		} catch (OdometerExceptions e) {
			e.printStackTrace();
		}
	}
	public void localize2() {
		setSpeeds(75,75);
		boolean right=false;
		boolean left=false;
		while(true) {
			if(getRightRValue() < BLACK) {
				
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
				leftMotor.rotate(convertDistance(Project.WHEEL_RAD, Project.DIST), true);
	      		rightMotor.rotate(convertDistance(Project.WHEEL_RAD, Project.DIST), false);
	        	setSpeeds(0, 0);
	        	left= false;
	        	right = false;
	        	break;
			}
		}
		
		//turn right
			leftMotor.setSpeed(75);
			rightMotor.setSpeed(75);
			leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 90), true);
			rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, 90), false);	

			
			setSpeeds(75,75);
			right=false;
			left=false;
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
					leftMotor.rotate(convertDistance(Project.WHEEL_RAD, Project.DIST), true);
		      		rightMotor.rotate(convertDistance(Project.WHEEL_RAD, Project.DIST), false);
		        	setSpeeds(0, 0);
		        	left= false;
		        	right = false;
		        	break;
				}
			}
		//odo.setXYT(30.48, 30.48, 90);
	}
	public float getRightRValue() {
		rightLightProvider.fetchSample(rightLightData, 0); // acquire data
		return rightLightData[0];
  }
	public float getLeftRValue() {
		leftLightProvider.fetchSample(leftLightData, 0); // acquire data
		return leftLightData[0];
  }
	public void localize() {
		
		//not facing wall
		setSpeeds(75,75);
		System.out.println("going forward");
		while (true) {
			if(leftLightData[0] < BLACK) {
				leftMotor.setSpeed(0);
				
				while(true) {
					if(rightLightData[0] < BLACK) {
						rightMotor.setSpeed(0);
						break;
					}
				}		
				break;
			}
			else if(rightLightData[0] < BLACK) {
				rightMotor.setSpeed(0);
				
				while(true) {
					if(leftLightData[0] < BLACK) {
						leftMotor.setSpeed(0);
						break;
					}
				}			
				break;
			}
		}
		
		//at 1st line 
		//reverse distance separating sensors from wheels
		//leftMotor.rotate(-convertDistance(Project.DIST), true);
	    //rightMotor.rotate(-convertDistance(Project.DIST), false);
		//turn right
		leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, -90), false);	
			
		//if facing wall 
		if(getDistance() != 255) {
			//turn 180
			leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 180), true);
			rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, -180), false);	
		}
		
		//else go forward
		setSpeeds(75,75);
		while (true) {
			if(leftLightData[0] < BLACK) {
				leftMotor.setSpeed(0);
					
				while(true) {
					if(rightLightData[0] < BLACK) {
						rightMotor.setSpeed(0);
						break;
					}
				}		
				break;
			}
			else if(rightLightData[0] < BLACK) {
				rightMotor.setSpeed(0);
				
				while(true) {
					if(leftLightData[0] < BLACK) {
						leftMotor.setSpeed(0);
						break;
					}
				}			
				break;
			}
		}
	//reverse distance separating sensors from wheels
		//leftMotor.rotate(-convertDistance(Project.DIST), true);
	    //rightMotor.rotate(-convertDistance(Project.DIST), false);
		
	//robot is at closest intersection update odo
		update();
		
		
	}
	
	/**
	 * This method is used to keep the robot moving forward in a straight line 
	 */
	public void straight() {
		while (true) {
			if(leftLightData[0] < BLACK) {
				leftMotor.setSpeed(0);
				
				while(true) {
					if(rightLightData[0] < BLACK) {
						rightMotor.setSpeed(0);
						break;
					}
				}	
				break;
			}
			else if(rightLightData[0] < BLACK) {
				rightMotor.setSpeed(0);
				
				while(true) {
					if(leftLightData[0] < BLACK) {
						leftMotor.setSpeed(0);
						break;
					}
				}
				break;
			}
			setSpeeds(75,75);
		}
	}
      
	/**
	 * This method sets the odometer values correctly based on starting corner
	 */
	public void update() {
		if(Wifi.startingCorner == 0) 
			odo.setXYT(1*TILE, 1*TILE, 0);
		
		else if (Wifi.startingCorner == 1) 
			odo.setXYT(14*TILE, 1*TILE, 0);
		
		else if (Wifi.startingCorner == 2) 
			odo.setXYT(14*TILE, 8*TILE, 180);
		
		else if (Wifi.startingCorner == 3) 
			odo.setXYT(14*TILE, 1*TILE, 180);
		
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
	 * Converting distance readings to an integer
	 * 
	 * @return integer
	 */
	public int getDistance() {
		usProvider.fetchSample(usData, 0); // acquire data
		return (int) (usData[0] * 100); // extract from buffer, cast to int
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
}

