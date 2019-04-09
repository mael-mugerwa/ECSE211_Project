package ca.mcgill.ecse211.Project.Testing;
import ca.mcgill.ecse211.Project.Project;
import ca.mcgill.ecse211.Project.Searching.Navigation;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;

public class Testing{
	// Motor Objects, Sensor Objects and Robot related parameters
		public EV3LargeRegulatedMotor leftMotor;		
		public EV3LargeRegulatedMotor rightMotor;
		
		public EV3MediumRegulatedMotor rotatingArmMotor;
		
		public EV3LargeRegulatedMotor gateMotor;
		
		public SampleProvider usSensorProvider;		
		public float[] usData;
		public SampleProvider rotatingLightSensorProvider;
		public float[] rotatingLightData;
		public SampleProvider rightLightSensorProvider;
		public float[] rightLightData;
		public  SampleProvider leftLightSensorProvider;
		public float[] leftLightData;
			
		Navigation nav;
		
		public  double L_WHEEL_RAD = 2.1;
		public  double WHEEL_RAD = 2.1;
		public  double R_WHEEL_RAD = 2.1;
		
		public double L_BLACK=0.08;
		public double R_BLACK=0.08;

		public  int FAST = 170;
		public  int SLOW = 130;

		public  double TILE = 30.48;

		public Testing(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			EV3MediumRegulatedMotor rotatingArmMotor,
			EV3LargeRegulatedMotor gateMotor,
			SampleProvider rotatingLightSensorProvider,
			SampleProvider leftLightSensorProvider,
			SampleProvider rightLightSensorProvider,
			SampleProvider usSensorProvider,
			Navigation navigation){
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
		
		this.rotatingArmMotor = rotatingArmMotor;
		rotatingArmMotor.setSpeed(FAST);
		
		this.gateMotor = gateMotor;
		gateMotor.setSpeed(FAST);
		
		this.usSensorProvider = usSensorProvider;
		this.usData = new float[1];
  	  
		this.rotatingLightSensorProvider = rotatingLightSensorProvider;
		this.rotatingLightData = new float[3];	
		
		this.leftLightSensorProvider = leftLightSensorProvider;
		this.leftLightData = new float[3];
		
		this.rightLightSensorProvider = rightLightSensorProvider;
		this.rightLightData = new float[3];
		
		this.nav = navigation;
	}
	

	 		/**
	  		* TEST Project.TRACK
	  		*/
	    	 public void testTrack() {	
	    	      // turn 90 degrees CLOCKWISE
	    	      leftMotor.setSpeed(100);
	    	      rightMotor.setSpeed(100);

	    	      leftMotor.rotate(convertAngle(WHEEL_RAD, Project.TRACK, 360.0*6), true);
	    	      rightMotor.rotate(-convertAngle(WHEEL_RAD, Project.TRACK, 360.0*6), false);
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
	    		 leftMotor.rotate(convertAngle(WHEEL_RAD, Project.TRACK, randomInt), true);
	    	     rightMotor.rotate(-convertAngle(WHEEL_RAD, Project.TRACK, randomInt), false);
	    	     
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
			 * Test Distance separating light sensors from wheels
			 */
	    	public void testDISTValue() {

		    	setSpeeds(-75, -75);
		    	while(true) {
		    	
		    		if(getRightRValue() < R_BLACK) {
		      			leftMotor.rotate((convertDistance(WHEEL_RAD, Project.DIST)), true);
		      			rightMotor.rotate((convertDistance(WHEEL_RAD, Project.DIST)), false);
		      			setSpeeds(0, 0);
		        	}
		    	}
		    	
	    	}
	    	public void getPower() {
	    		//float current = LocalEV3.ev3.getPower().get
	    	}
	    	
	 
			/**
			 * Test the R value for a black line by having the  black R value
			 */
	    	public void testRightBlackValue() {
	    		leftMotor.setSpeed(100);
	    	    rightMotor.setSpeed(100);
Sound.setVolume(100);
	    	      leftMotor.rotate(convertAngle(WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      rightMotor.rotate(-convertAngle(WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      while(true) 	{        	
	        	System.out.println(getRightRValue());
	        	if(getRightRValue()<R_BLACK)
	        		Sound.beep();
	    	      }
	    	}	
	    	
			/**
			 * Test the R value for a black line by having the  black R value
			 */
	    	public void testLeftBlackValue() {
	    		leftMotor.setSpeed(100);
	    	    rightMotor.setSpeed(100);
Sound.setVolume(100);
	    	      leftMotor.rotate(convertAngle(WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      rightMotor.rotate(-convertAngle(WHEEL_RAD, Project.TRACK, 360.0), true);
	    	      while(true) 	{        	
	        	System.out.println(getLeftRValue());
	        	if(getLeftRValue()<L_BLACK)
	        		Sound.beep();
	    	      }
	    	}	
	    	
	    	/**
	    	 * Test if the robot is going perfectly forward by having it go forward the length of 5 tile sizes
	    	 */
	    	public void testGoingStraight() {
	    		leftMotor.setSpeed(100);
			    rightMotor.setSpeed(100);
			    leftMotor.rotate((convertDistance(WHEEL_RAD, 14*30.48)), true);
		  		rightMotor.rotate((convertDistance(WHEEL_RAD, 14*30.48)), false);
		    	
	    	}
	    	
	    	/**
	    	 * Test Wheel Radius by having each wheel complete 5 turns
	    	 *  and making sure that the orange piece placed at the end of each wheel is at the same orientation it started at.
	    	 */
	    	public void testLeftWheelRadius() {
	    		leftMotor.setSpeed(100);
			    leftMotor.rotate((convertDistance(L_WHEEL_RAD, 5*2*Math.PI*L_WHEEL_RAD)), true);
	    	}
	    	
	    	/**
	    	 * Test Wheel Radius by having each wheel complete 5 turns
	    	 *  and making sure that the orange piece placed at the end of each wheel is at the same orientation it started at.
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
	 
		public float getRightRValue() {
			rightLightSensorProvider.fetchSample(rightLightData, 0); // acquire data
			return rightLightData[0];
	  }
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
	
	public void setSpeeds(int lSpd, int rSpd) {
		leftMotor.setSpeed(lSpd);
		rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			leftMotor.backward();
		else
			leftMotor.forward();
		if (rSpd < 0)
			rightMotor.backward();
		else
			rightMotor.forward();
	}
	
}

