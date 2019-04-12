package ca.mcgill.ecse211.Project.Searching;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.robotics.SampleProvider;
import java.util.ArrayList;

import ca.mcgill.ecse211.Project.Project;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;

/**
 * Can Scanner class housing all scanning methods and color detection methods. 
 * @author Mael
 *
 */
public class CanScanner  {
	
	private static CanScanner canScanner = null; // Returned as singleton
	
	/**
	 * Optimal distance to scan a can
	 */
	final static int TARGET_DIST = 2;
	/**
	 * Distance to reverse before picking up can
	 */
	final static int BACKWARD = 15;
	/**
	 * Distance need to go forward to pick up can
	 */
	final static int FORWARD = 22;
	
	/**
	 * The number of cans inside the robot. Used to know if the robot is full and should stop searching for cans
	 */
	public static int numberOfCansCollected = 0;
	/**
	 * Motor speed when picking up cans and unloading them
	 */
	final static int SPEED = 100;
	/**
	 * left motor object
	 */
	private EV3LargeRegulatedMotor leftMotor;
	/**
	 * right motor object
	 */
	private EV3LargeRegulatedMotor rightMotor;	
	/**
	 * arm motor object. This is the rotating arm at the end of which is the light sensor used for can scanning
	 */
	private EV3MediumRegulatedMotor arm;
	/**
	 * gate motor object. used to open and close the gate to pick up or move cans aside
	 */
	private EV3LargeRegulatedMotor gate;
	
	/**
	 * light sensor sample provider object. This is the sensor that scans the cans
	 */
	private SampleProvider lightProvider;
	/**
	 * Array where we store can color readings
	 */
	private float[] lightData;
	
	/**
	 * Constructor 
	 * @param leftMotor
	 * @param rightMotor
	 * @param rotatingArmMotor
	 * @param gateMotor
	 * @param rotatingLightSensorProvider
	 * @param usSensorProvider
	 */
	public CanScanner (EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor,	
			EV3MediumRegulatedMotor rotatingArmMotor,
			EV3LargeRegulatedMotor gateMotor,
			SampleProvider rotatingLightSensorProvider,
			SampleProvider usSensorProvider) {
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;		
		
		this.arm = rotatingArmMotor;
		arm.setSpeed(Project.FORWARD_SPEED);
		
		this.gate = gateMotor;
		gate.setAcceleration(1000);
		gate.setSpeed(75);
		
  	  
		this.lightProvider = rotatingLightSensorProvider;
		this.lightData = new float[rotatingLightSensorProvider.sampleSize()];	
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
	

      /**
       * This method scans a can of unknown color and calculates its mean readings for each channel 
       * 
       * @return float[]
       */
      public float[] scanCan() {
    	  
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
          float SmeanR = getRegMean( sampleR );
          float SmeanG = getRegMean( sampleG );
          float SmeanB = getRegMean( sampleB );
          
          //Get final standardized means using formula from lab instructions and add them to return array
          ret[0]= (float) (SmeanR/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
          ret[1]= (float) (SmeanG/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
          ret[2]= (float) (SmeanB/Math.sqrt(SmeanR*SmeanR + SmeanG*SmeanG + SmeanB*SmeanB));
         
          return ret;
      }
      
      /**
       * This method calculates the mean for each color value from the values obtained after scanning a can
       *
       * @param colorArray
       * @return
       */
      public float getRegMean(ArrayList<Float> colorArray) {
     	  float sum=0;
           for (float val : colorArray) 
           	sum += val;
           return sum/colorArray.size();
       }
             
      /**
       *This method is called in the run method to determine a can's color by finding the closest mean values to a scanned can of unknown color. 
       *This is done by calculating the Euclidean distance using mean values previously obtained for each can color through testing.
       *If a can is of the desired color, it is picked up and weighed otherwise, it is moved aside
       * 
       * @return boolean
       */
      public boolean colorDetection() {
    	            
    	  boolean pickedUp = false;//boolean to know if a can was picked up
    	  
    	  float[] ColorArray = this.scanCan();
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

    		  this.pickUpCan(index);
    		  pickedUp = true;//we picked up the can
    		  numberOfCansCollected++;
    	  }
    	  else {//can is not target color
    		  this.moveCanAside();
    	  } 
    	  
          return pickedUp;//return if the can was picked up
      }      
         
      /**
       * This method turns the gate motor into an unregulated motor to weigh a can. 
       * By setting the power to 20, 
       * @return
       */
      public int getWeight() {
    	  int ret;
    	  
    	  gate.stop();
    	  gate.close();
    	  UnregulatedMotor gateForWeighing =
    				new UnregulatedMotor(LocalEV3.get().getPort("B"));
    	  gateForWeighing.setPower(20);
    	  
    	  int initialTacho = gateForWeighing.getTachoCount();
    	  long initialTime = System.currentTimeMillis();
    	  
    	  gateForWeighing.forward();
    	  
    	  while(System.currentTimeMillis()-initialTime < 2000) {
    		  
    	  }

    	  gateForWeighing.stop();
    	  int finalTacho = gateForWeighing.getTachoCount();
    	  gateForWeighing.close();
    	   
    	  if(Math.abs(finalTacho-initialTacho) > 70) {
				ret = 500;
				System.out.println("LIGHT");
    	  }
    	  else {
				ret = 1000;
				System.out.println("HEAVY");
    	  }
    	  
    	  gate = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
    	  gate.setSpeed(150);
    	  gate.rotate(100-Math.abs(finalTacho-initialTacho));
    	  
    	  return ret;

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
      
      /**
       * This is how we obtained the mean values for each can color
       */
      public void testMeans() {
    	  float[] ColorArray;
    	  
		  ColorArray = this.scanCan();

    	  float SRmi = ColorArray[0];
          float SGmi = ColorArray[1];
          float SBmi = ColorArray[2];
          
    	  for(int i=0; i<10; i++) {
    		  
    		  ColorArray = this.scanCan();
        	  
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
      public void pickUpCan(int colorBeeps) {
    	  int weight;
    	  
    	  //reverse 
    	  backwardsSetDistance(BACKWARD, SPEED);
    		
    	  //open trap door
    	  gate.rotate(-100);    	  
    	  
    	  //go forward
    	  forwardsSetDistance(FORWARD, SPEED);
    	  
    	  //close trap door
    	  weight = getWeight();
    	  
    	  for(int i=0; i<colorBeeps+1; i++) {
				Sound.playTone(500, weight);
				try {
					Thread.sleep(250);
				} catch (InterruptedException e) {
					//do nothing
				}
    	  }
    	  
    	  
      }
      
      /**
       * this method opens the gate all the way. This is called before traveling to the start corner to unload the cans. 
       * This is done because the robot does not have enough space to open the gate when it reaches the starting corner
       */
      public void openDoor() {
    	  gate.rotate(-200);
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

  	/**
  	 * get current instance of canScanner
  	 * @param leftMotor
  	 * @param rightMotor
  	 * @param arm
  	 * @param gate
  	 * @param lightProvider
  	 * @param usProvider
  	 * @return
  	 */
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


