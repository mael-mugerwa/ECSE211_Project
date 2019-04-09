package ca.mcgill.ecse211.Project.Searching;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.ArrayList;

import ca.mcgill.ecse211.Project.Project;
import lejos.hardware.Sound;

public class CanScanner  {
	
	private static CanScanner canScanner = null; // Returned as singleton
	
	//Optimal distance to scan a can
	final static int TARGET_DIST = 2;
	//Distance to reverse before picking up can
	final static int BACKWARD = 15;
	//Distance need to go forward to pick up can
	final static int FORWARD = 25;
	
	public static int numberOfCansCollected = 0;
	//Motor speed when picking up cans and unloading them
	final static int SPEED = 100;

	private EV3LargeRegulatedMotor leftMotor;
	private EV3LargeRegulatedMotor rightMotor;	
	private EV3MediumRegulatedMotor arm;
	private EV3LargeRegulatedMotor gate;
	
	private SampleProvider lightProvider;
	private float[] lightData;
	
	private SampleProvider usProvider;
	private float[] usData;	
	
	
	public CanScanner (EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			EV3MediumRegulatedMotor rotatingArmMotor,
			EV3LargeRegulatedMotor gateMotor,
			SampleProvider rotatingLightSensorProvider,
			SampleProvider usSensorProvider) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
		
		this.arm = rotatingArmMotor;
		arm.setSpeed(Project.FAST);
		
		this.gate = gateMotor;
		gate.setAcceleration(1000);
		gate.setSpeed(75);
		
		this.usProvider = usSensorProvider;
		this.usData = new float[usSensorProvider.sampleSize()];
  	  
		this.lightProvider = rotatingLightSensorProvider;
		this.lightData = new float[rotatingLightSensorProvider.sampleSize()];	
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
	 * Get R color reading
	 * @return float
	 */
	public float getR() {
		lightProvider.fetchSample(lightData, 0); // acquire data
		return lightData[0];
	}	
	
	/**
	 * Get G color reading
	 * @return 
	 */
	public float getG() {
		lightProvider.fetchSample(lightData, 0); // acquire data
		return lightData[1];
	}	
	
	/**
	 * Get B color reading
	 * @return 
	 */
	public float getB() {
		lightProvider.fetchSample(lightData, 0); // acquire data
		return lightData[2];
	}	
	
	
      /*
      public void getClosetoCan() {
    	  navigation.setSpeeds(25,25);
    	  while(true) {
    		  if(getDistance()<TARGET)
    			  break;
    	  }
    	  navigation.setSpeeds(0,0);
      }
      */      

      /**
       * This method scans a can of unknown color and calculates its mean readings for each channel 
       * 
       * @return float[]
       */
      public float[] scanCan3() {
    	  
    	  //return array where the sample can's means will be stored
    	 float[] ret = new float[3]; 
    	
    	  //ArrayLists where we'll store each color reading from the sample separated by channel (RGB)
    	  ArrayList<Float> sampleR = new ArrayList<Float>(1);
    	  ArrayList<Float> sampleG = new ArrayList<Float>(1);          
    	  ArrayList<Float> sampleB = new ArrayList<Float>(1);   
    	  
    	  arm.setSpeed(150);
          arm.rotate(-100, true);//rotate arm
           
          while (arm.isMoving()) {
        	  //take color measurements
        	  sampleR.add(getR());//store in arraylist based on channel
        	  sampleG.add(getG());
        	  sampleB.add(getB());
          }
                   
          arm.rotate(100, false);//rotate arm back

          //Calculate the sample means for each channel
          float SmeanR = getRegMean2( sampleR );
          float SmeanG = getRegMean2( sampleG );
          float SmeanB = getRegMean2( sampleB );
          
          //Get final standardized means using formula from lab instructions and add them to return array
          ret[0]= (float) (SmeanR/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
          ret[1]= (float) (SmeanG/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
          ret[2]= (float) (SmeanB/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
         
          return ret;
      }
      
      /**
       * This method calculates the mean of color reading array
       * 
       * @param colorArray
       * @return
       */
      public float getRegMean2(ArrayList<Float> colorArray) {
     	  float sum=0;
           for (float val : colorArray) 
           	sum += val;
           return sum/colorArray.size();
       }
             
      /**
       *This method is called in the run method to determine a can's color
       * 
S       * @return boolean
       */
      public boolean colorDetection() {
    	            
    	  boolean pickedUp = false;//boolean to know if a can was picked up
    	  
    	  float[] ColorArray = this.scanCan3();
    	//Yellow Can average mean values for each channel 
          //Determined by experimentation
          float YRmi = (float)0.8560424;
          float YGmi = (float)0.4211464;
          float YBmi = (float)0.29550308; 
          
        //Blue Can average mean values for each channel 
          //Determined by experimentation
          float BRmi = (float)0.5742459;
          float BGmi = (float)0.48236978;
          float BBmi = (float)0.6516238;
          
        //Red Can average mean values for each channel 
          //Determined by experimentation
          float RRmi = (float)0.9505286;
          float RGmi = (float)0.1434068;
          float RBmi = (float)0.272362; 
          
        //Green Can average mean values for each channel 
          //Determined by experimentation
          float GRmi = (float)0.634494;
          float GGmi = (float)0.5799018;
          float GBmi = (float)0.50631493;
                   
          //Sample Can mean values
          float SRmi = ColorArray[0];
          float SGmi = ColorArray[1];
          float SBmi = ColorArray[2];
          
          //array where we store euclidean  distance for each color
          float[] diff = new float[4];
          
          //Yellow
          diff[2] = (float) Math.sqrt(Math.pow((SRmi-YRmi),2)+Math.pow((SGmi-YGmi),2)+Math.pow((SBmi-YBmi),2));
          
          //Red
          diff[3] = (float) Math.sqrt(Math.pow((SRmi-RRmi),2)+Math.pow((SGmi-RGmi),2)+Math.pow((SBmi-RBmi),2));

          //Green
          diff[1] = (float) Math.sqrt(Math.pow((SRmi-GRmi),2)+Math.pow((SGmi-GGmi),2)+Math.pow((SBmi-GBmi),2));

          //Blue
          diff[0] = (float) Math.sqrt(Math.pow((SRmi-BRmi),2)+Math.pow((SGmi-BGmi),2)+Math.pow((SBmi-BBmi),2));

          int index = 0;
          float min = diff[index];
          for (int i=1; i<diff.length; i++){
              if (diff[i] < min ){
                  min = diff[i];
                  index = i;
              }           
          }
          
          if(index==2) {
        	  System.out.println("Yellow");
          }
          else if(index==3) {
        	  System.out.println("Red");
          }
          else if(index==1) {
        	  System.out.println("Green");
          }
          else if(index==0) {
        	  System.out.println("Blue");
          }
          
    	  if(true) {//pick up every can
    		  Sound.setVolume(100);    		 

    		  this.pickUpCan();
    		  pickedUp = true;//we picked up the can
    		  numberOfCansCollected++;
    	  }
    	  else {//can is not target color
    		  this.moveCanAside();
    	  } 
    	  
          return pickedUp;//return if the can was picked up
      }      
         
      /**
       * This method moves can aside by opening the door, the can is moved aside and the robot can continue moving forward
       */
      public void moveCanAside() {
    	  //open trap door
    	  gate.rotate(-120);    	  
    	  //can has been moved aside
    	  //close trap door
    	  gate.rotate(130); // to make sure gate is closed
      }
      
      
      public void testMeans() {
    	  float[] ColorArray;
    	  
		  ColorArray = this.scanCan3();

    	  float SRmi = ColorArray[0];
          float SGmi = ColorArray[1];
          float SBmi = ColorArray[2];
          
    	  for(int i=0; i<10; i++) {
    		  
    		  ColorArray = this.scanCan3();
        	  
    	    	//Sample Can mean values
    	           SRmi = (SRmi +ColorArray[0])/2;
    	           SGmi = (SGmi + ColorArray[1])/2;
    	           SBmi = (SBmi + ColorArray[2])/2;
    	  }
    	  
    	  System.out.println(SRmi);
    	  System.out.println(SGmi);
    	  System.out.println(SBmi);

      }
      
      /**
       * This method pick ups a can by opening the door and collecting the can
       */
      public void pickUpCan() {
    	  
    	  //reverse 
    	  backwardsSetDistance(BACKWARD, SPEED);
    		
    	  //open trap door
    	  gate.rotate(-110);    	  
    	  
    	  //go forward
    	  forwardsSetDistance(FORWARD, SPEED);
    	  
    	  //close trap door
    	  gate.rotate(110); 
    	  
      }
      
      
      /**
       * This method unloads all the cans by opening the door and reversing
       * called by travelTo()
       */
      public void unloadingCans() {
    	  //open trap door
    	  gate.rotate(-130);
    	  
    	  //reverse 
    	  backwardsSetDistance(2*BACKWARD, SPEED);
    	  
    	  //close trap door
    	  gate.rotate(130); 
      }
      
      /**
  	 * This method makes the robot move forwards a set distance
  	 * 
  	 * @param Distance
  	 * @param speed
  	 */
  	public void forwardsSetDistance(double Distance, int speed) {
  		leftMotor.setSpeed(speed);
  		rightMotor.setSpeed(speed);
  		leftMotor.rotate(Navigation.convertDistance(Distance), true);
  		rightMotor.rotate(Navigation.convertDistance(Distance), false);
  	}

  	/**
  	 * This method makes the robot move backwards a set distance
  	 * 
  	 * @param Distance
  	 * @param speed
  	 */
  	public void backwardsSetDistance(double Distance, int speed) {
  		leftMotor.setSpeed(speed);
  		rightMotor.setSpeed(speed);
  		leftMotor.rotate(-(Navigation.convertDistance(Distance)), true);
  		rightMotor.rotate(-(Navigation.convertDistance(Distance)), false);
  	}

	public static CanScanner getCanScanner(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			EV3MediumRegulatedMotor arm,
			EV3LargeRegulatedMotor gate,
			SampleProvider lightProvider,
			SampleProvider usProvider) {

		if (canScanner != null) { // Return existing object
		      return canScanner;
		} else { // create object and return it
		      canScanner = new CanScanner(leftMotor, rightMotor, arm, gate, lightProvider, usProvider);
		      return canScanner;
		}
	}
      

}


