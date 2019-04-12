package ca.mcgill.ecse211.Project;
import ca.mcgill.ecse211.Project.Localization.LightLocalization;
import ca.mcgill.ecse211.Project.Localization.UltrasonicLocalization;
import ca.mcgill.ecse211.Project.Odometer.Odometer;
import ca.mcgill.ecse211.Project.Odometer.OdometerExceptions;
import ca.mcgill.ecse211.Project.Searching.CanScanner;
import ca.mcgill.ecse211.Project.Searching.Navigation;
import ca.mcgill.ecse211.Project.Testing.Testing;
import ca.mcgill.ecse211.Project.Wifi.Wifi;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.UnregulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This is our main class. In it, we declare all our motor and sensor objects creating instances of
 * each object, the port it is connected to and its sample provider if it is a sensor.
 * We also outline all the robot related parameters
 * These parameters are used by all of the other classes. 
 * Additionally, we create our threads in this class. The odometer thread is always running the other threads are created when needed
 *   
 * @author Mael Mugerwa
 */
public class Project{
	
	/**
	 * left motor object
	 */
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	/**
	 * right motor object
	 */
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	/**
	 * rotating motor object. This is the motor that rotates the light sensor arm to scan a can
	 */
	public static final EV3MediumRegulatedMotor rotatingArmMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	/**
	 * gate motor object. This motor opens and closes the gate when picking up cans
	 */
	public static final EV3LargeRegulatedMotor gateMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	/**
	 * front facing ultrasonic sensor object. Used for us localization and our searching routine
	 */
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
	public static final SampleProvider usProvider= usSensor.getMode("Distance");
	
	/**
	 * rotating light sensor object. Used for scanning cans
	 */
	public static final EV3ColorSensor rotatingLightSensor = new EV3ColorSensor(SensorPort.S3);
	public static final SampleProvider rotatingLightProvider = rotatingLightSensor.getMode("RGB");

	/**
	 * right light sensor object. Used for light localization
	 */
	public static final EV3ColorSensor rightLightSensor = new EV3ColorSensor(SensorPort.S1);
	public static final SampleProvider rightLightProvider = rightLightSensor.getMode("RGB");

	/**
	 * left light sensor object. Used for light localization
	 */
	public static final EV3ColorSensor leftLightSensor = new EV3ColorSensor(SensorPort.S4);
	public static final SampleProvider leftLightProvider = leftLightSensor.getMode("RGB");
		
	/**
	 * Wheel radius parameter
	 */
	public static final double WHEEL_RAD = 2.1;
	/**
	 * Wheel track parameter
	 */
    public static final double TRACK = 16;
    /**
     * This is the distance separating the wheels from both light sensors
     */
	public static final double DIST = 5.27;
	/**
	 * This is the light sensor R reading that corresponds to a black line
	 */
	public static final double BLACK = 0.08;
	/**
	 *  Speed parameter when going forward
	 */
	public static final int FORWARD_SPEED = 170;
	/**
	 *  Speed parameter when rotating
	 */
	public static final int ROTATE_SPEED = 130;
	/**
	 * 
	 * Tile size parameter
	 */
	public static final double TILE = 30.48;


	/**
	 * Main method where we create our Odometer, CanScanner, Navigation, Localization, Testing objects and start the Odometer, Search Beeping and main threads
	 * The Odometer thread runs continuously to enable the robot to always know where it is. The Search Beeping thread is used to make the robot beep once it enters the search zone.
	 * We need this thread because when searching our robot travels to the center of the search zone. While traveling to that point the robot might pickup cans making it impossible to beep when reaching the center.
	 * The final thread is our main thread where we call all our methods: running Wifi, Localizing, Traveling using Navigation, Searching...
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
	    
		  /**
		   * Odometer related objects
		   */
		  final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor);

		  /**
		   * CanScanner related objects
		   */
		  final CanScanner canScanner = CanScanner.getCanScanner(leftMotor, rightMotor, rotatingArmMotor, gateMotor, rotatingLightProvider, usProvider);
		  
		  /**
		   * Start odometer thread
		   */
	      Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      
	      /**
	       * Navigation related objects. This class also contains the searching method
	       */
	      final Navigation navigation = Navigation.getNavigation(leftMotor, rightMotor, usProvider, canScanner);
	      
	      /**
	       *  UltrasonicLocalization related object
	       */
		  final UltrasonicLocalization usLocalization = new UltrasonicLocalization(leftMotor, rightMotor, usSensor);
		  /**
		   * LightLocalization related object
		   */
	      final LightLocalization lightLocalization = new LightLocalization(leftMotor, rightMotor, usProvider, leftLightProvider, rightLightProvider, navigation);
	      /**
	       * Testing related object. This class houses all our test to make testing simple
	       */
	      final Testing test = new Testing(leftMotor, rightMotor, rotatingArmMotor, gateMotor, rotatingLightProvider, leftLightProvider, rightLightProvider, usProvider, navigation, canScanner);
	      
	      /**
	       *  Wifi related objects. This is where we parse parameters received from the server
	       */
	      final Wifi wifi = new Wifi();
	      	      	      
	      /**
	       * Spawn new thread just for beeping when reaching search zone
	       */
	      (new Thread() {
	    	  public void run() {
	    		  while(true) {
	    			  if(Wifi.searchZone_LL_x != 0.0 && Wifi.searchZone_LL_y !=0.0) {
	    				  boolean x = ((Wifi.searchZone_LL_x*30.48 <= odometer.getX()) && (odometer.getX() <= Wifi.searchZone_UR_x*30.48 ));
	    				  boolean y = ((Wifi.searchZone_LL_y*30.48 <= odometer.getY()) && (odometer.getY() <= Wifi.searchZone_UR_y*30.48 ));
	    				  //System.out.println(x+" "+y);
	    				  if(x && y) {
		    				  for(int i=0; i<3; i++) {
		    					  Sound.playTone(500, 500);
		  							try {
		  								Thread.sleep(250);
		  							} catch (InterruptedException e) {
		  								//do nothing
		  							}
		    				  }
		    				  break;
		    			  }
	    			  }
	    			 
	    		  }
	    		  
	    	  }
	      }).start();
	      
	      
	      /**
	       * Spawn the main Thread for our project. Calls methods from all our classes
	       */
	      (new Thread() {
			public void run() {
				Sound.setVolume(100);
				//run wifi				
				wifi.run();
				
				//us localization using falling edge
				usLocalization.fallingEdge();	
				//light localization
				lightLocalization.localize();
				
				//beeping once localization is over
				for(int i=0; i<3; i++) {
					Sound.playTone(500, 500);
					try {
						Thread.sleep(250);
					} catch (InterruptedException e) {
						//do nothing
					}
				}
				
				//travel to tunnel entrance without using correction 
				navigation.travelToTunnelEntrance(false);
				//relocalize before crossing tunnel
				double angle = lightLocalization.reLocalize();
				//correct the odometer
				odometer.setXYT(Wifi.tunnelEntrance_x*30.48, Wifi.tunnelEntrance_y*30.48, angle);
				//travel to tunnel exit using correction
				navigation.travelToTunnelExit(true);
				
				//travel to search center
				navigation.travelToSearchCenter();
				
				//search
				navigation.searchRoutine();	
				
				//travel to tunnel exit without correction
				navigation.travelToTunnelExit(false);
				
				//2nd relocalization method
				angle = lightLocalization.reLocalizeAgain();
				//odometer correction
				odometer.setXYT(Wifi.tunnelExit_x*30.48, Wifi.tunnelExit_y*30.48, angle);
				//travel to tunnel entrance using correciton
				navigation.travelToTunnelEntrance(true);
				//call first relocolization method
				angle = lightLocalization.reLocalize();
				//odometer correction
				odometer.setXYT(Wifi.tunnelEntrance_x*30.48, Wifi.tunnelEntrance_y*30.48, angle);
				
				//travel to starting corner to unload cans
				navigation.travelToStartCorner();
				
				//final beeps
				for(int i=0; i<5; i++) {
					Sound.playTone(500, 500);
					try {
						Thread.sleep(250);
					} catch (InterruptedException e) {
						//do nothing
					}
				}
			
				
	        }
	      }).start();	       
	      
	      while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	      System.exit(0);
	}
	  
}