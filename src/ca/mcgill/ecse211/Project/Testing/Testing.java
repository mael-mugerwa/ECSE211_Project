
package ca.mcgill.ecse211.Project.Testing;
import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Searching.CanScanner;
import ca.mcgill.ecse211.Project.Searching.Navigation;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.motor.BaseRegulatedMotor;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.RegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;

/**
 * This is our testing class where we can call multiple methods to test robot parameters and more
 * @author Mael
 *
 */
public class Testing{
		/**
		 * left motor object
		 */
		public EV3LargeRegulatedMotor leftMotor;	
		/**
		 * right motor object
		 */
		public EV3LargeRegulatedMotor rightMotor;
		
		/**
		 * rotating arm motor object. used for can scanning
		 */
		public EV3MediumRegulatedMotor rotatingArmMotor;
		
		/**
		 * gate motor object
		 */
		public EV3LargeRegulatedMotor gateMotor;
		
		/**
		 * us sensor provider
		 */
		public SampleProvider usSensorProvider;		
		public float[] usData;
		/**
		 * rotating light sensor provider. used for can scanning
		 */
		public SampleProvider rotatingLightSensorProvider;
		public float[] rotatingLightData;
		/**
		 * right light sensor provider. used for localizing
		 */
		public SampleProvider rightLightSensorProvider;
		public float[] rightLightData;
		/**
		 * left light sensor provider. used for localizing
		 */
		public  SampleProvider leftLightSensorProvider;
		public float[] leftLightData;
			
		/**
		 * navigation object
		 */
		Navigation nav;
		/**
		 * can Scaneer object
		 */
		CanScanner canScanner;
		
		/**
		 * Left wheel radius parameter
		 */
		public  double L_WHEEL_RAD = 2.1;
		/**
		 * right wheel radius parameter
		 */
		public  double R_WHEEL_RAD = 2.1;
		
		/**
		 * This is the left light sensor R reading that corresponds to a black line
		 */
		public double L_BLACK=0.08;
		/**
		 * This is the right light sensor R reading that corresponds to a black line
		 */
		public double R_BLACK=0.08;

		/**
		 * Constructor
		 * @param leftMotor
		 * @param rightMotor
		 * @param rotatingArmMotor
		 * @param gatemotor
		 * @param rotatingLightSensorProvider
		 * @param leftLightSensorProvider
		 * @param rightLightSensorProvider
		 * @param usSensorProvider
		 * @param navigation
		 * @param canScanner
		 */
		public Testing(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			EV3MediumRegulatedMotor rotatingArmMotor,
			EV3LargeRegulatedMotor gatemotor,
			SampleProvider rotatingLightSensorProvider,
			SampleProvider leftLightSensorProvider,
			SampleProvider rightLightSensorProvider,
			SampleProvider usSensorProvider,
			Navigation navigation,
			CanScanner canScanner){
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
		
		this.rotatingArmMotor = rotatingArmMotor;
		rotatingArmMotor.setSpeed(150);
		
		this.gateMotor = gatemotor;
		 gatemotor.setSpeed(150);
		
		this.usSensorProvider = usSensorProvider;
		this.usData = new float[1];
  	  
		this.rotatingLightSensorProvider = rotatingLightSensorProvider;
		this.rotatingLightData = new float[3];	
		
		this.leftLightSensorProvider = leftLightSensorProvider;
		this.leftLightData = new float[3];
		
		this.rightLightSensorProvider = rightLightSensorProvider;
		this.rightLightData = new float[3];
		
		this.nav = navigation;
		this.canScanner = canScanner;
	}
	

	 		/**
	  		* TEST Project.TRACK value to make sure it is correct by making the robot rotate on itself until it completes 6 turns
	  		*  if the robot is not in the same position as the one it started in the track is wrong and needs to be changed
	  		*/
	    	 public void testTrack() {	
	    	     
	    	      leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0*6), true);
	    	      rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0*6), false);
	    	   }
	    	 
	    	 /**
	    	  * This method is to test the US sensor distance readings
	    	  * Used to get the target distance to start can scanning
	    	  */
	    	 public void testUsDistance() {
	    		 while(true) {
	    			 LCD.drawInt(getDistance(), 0, 0);
	    		 }
	    	 }
	    	 
	    	 /**
	    	  * Test the weighing method by continuously calling the getWeight method until the tester stops the robot 
	    	  */
	    	 public void testWeighing() {
	    		 while(true) {//to scan multiple times    			
	    			
	    			 Sound.beep();
	    			Button.waitForAnyPress();//to swap cans
	    			canScanner.getWeight();//getWeight
	    			
	    		 }	    		 
	    	 }
	    	 
	    	 /**
	    	  * Test the faceTheta method by having the robot tunr at a random angle then correcting itself to face at specified theta
	    	  * @param theta
	    	  */
	    	 public void testFaceTheta(double theta) {
	    		 leftMotor.setSpeed(150);
	    		 rightMotor.setSpeed(150);
	    		 
	    		 double randomDouble = Math.random();
	    		 randomDouble = randomDouble * 360 + 1; // random number between 0 and 360
	    		 int randomInt = (int) randomDouble;
	    		 
	    		 //turn at random angle
	    		 leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, randomInt), true);
	    	     rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, randomInt), false);
	    	     
	    	     //face theta
	    	     nav.faceTheta(theta);
	    	     
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
			 * Test Distance separating light sensors from wheels by having the robot reverse until it detects a black line then moving forwards the DIST parameter value.
			 * If the center of the wheels is not on the detected black line, the DIST value is wrong
			 */
	    	public void testDISTValue() {

		    	nav.setSpeeds(-75, -75);
		    	while(true) {
		    	
		    		if(getRightRValue() < R_BLACK) {
		      			leftMotor.rotate((convertDistance(Project.WHEEL_RAD, Project.DIST)), true);
		      			rightMotor.rotate((convertDistance(Project.WHEEL_RAD, Project.DIST)), false);
		      			nav.setSpeeds(0, 0);
		        	}
		    	}
		    	
	    	}
	    
	 
			/**
			 * Test the R value for a black line by having the robot turn on itself printing out right light sensor values. Using 
			 * EV3 control to copy these values, by placing them in excel the black R value is easily seen on a graph
			 */
	    	public void testRightBlackValue() {
	    		leftMotor.setSpeed(100);
	    	    rightMotor.setSpeed(100);
Sound.setVolume(100);
	    	      leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      while(true) 	{        	
	        	System.out.println(getRightRValue());
	        	if(getRightRValue()<R_BLACK)
	        		Sound.beep();
	    	      }
	    	}	
	    	
	    	/**
			 * Test the L value for a black line by having the robot turn on itself printing out left light sensor values. Using 
			 * EV3 control to copy these values, by placing them in excel the black L value is easily seen on a graph
			 */
	    	public void testLeftBlackValue() {
	    		leftMotor.setSpeed(100);
	    	    rightMotor.setSpeed(100);
Sound.setVolume(100);
	    	      leftMotor.rotate(convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      rightMotor.rotate(-convertAngle(Project.WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      while(true) 	{        	
	        	System.out.println(getLeftRValue());
	        	if(getLeftRValue()<L_BLACK)
	        		Sound.beep();
	    	      }
	    	}	
	    	
	    	/**
	    	 * Test if the robot is going perfectly forward by having it go forward the length of 14 tile sizes
	    	 */
	    	public void testGoingStraight() {
	    		leftMotor.setSpeed(100);
			    rightMotor.setSpeed(100);
			    leftMotor.rotate((convertDistance(Project.WHEEL_RAD, 14*30.48)), true);
		  		rightMotor.rotate((convertDistance(Project.WHEEL_RAD, 14*30.48)), false);
		    	
	    	}
	    	
	    	/**
	    	 * Test Wheel Radius by having the left wheel complete 5 turns
	    	 *  and making sure that the orange piece placed at the end of the wheel is at the same orientation it started at.
	    	 */
	    	public void testLeftWheelRadius() {
	    		leftMotor.setSpeed(100);
			    leftMotor.rotate((convertDistance(L_WHEEL_RAD, 5*2*Math.PI*L_WHEEL_RAD)), true);
	    	}
	    	
	    	/**
	    	 * Test Wheel Radius by having the right wheel complete 5 turns
	    	 *  and making sure that the orange piece placed at the end of the wheel is at the same orientation it started at.
	    	 */
	    	public void testRightWheelRadius() {
	    		rightMotor.setSpeed(100);
			    rightMotor.rotate((convertDistance(R_WHEEL_RAD, 5*2*Math.PI*R_WHEEL_RAD)), true);
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
	  * return right light sensor values
	  * @return
	  */
		public float getRightRValue() {
			rightLightSensorProvider.fetchSample(rightLightData, 0); // acquire data
			return rightLightData[0];
	  }
		/**
		 * 	return left light sensor values
		 * @return
		 */
		public float getLeftRValue() {
			leftLightSensorProvider.fetchSample(leftLightData, 0); // acquire data
			return leftLightData[0];
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


