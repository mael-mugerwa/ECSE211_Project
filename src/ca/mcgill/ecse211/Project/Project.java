package ca.mcgill.ecse211.Project;
import ca.mcgill.ecse211.Project.Localization.LightLocalization;
import ca.mcgill.ecse211.Project.Localization.UltrasonicLocalization;
import ca.mcgill.ecse211.Project.Searching.CanScanner;
import ca.mcgill.ecse211.Project.Searching.Navigation;
import ca.mcgill.ecse211.Project.Testing.Testing;
import ca.mcgill.ecse211.Project.Wifi.Wifi;
import ca.mcgill.esce211.Project.Odometer.Odometer;
import ca.mcgill.esce211.Project.Odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
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
	
	// Motor Objects, Sensor Objects 
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	public static final EV3MediumRegulatedMotor rotatingArmMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	public static final EV3LargeRegulatedMotor gateMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
	public static final SampleProvider usProvider= usSensor.getMode("Distance");
	
	public static final EV3ColorSensor rotatingLightSensor = new EV3ColorSensor(SensorPort.S3);
	public static final SampleProvider rotatingLightProvider = rotatingLightSensor.getMode("RGB");

	public static final EV3ColorSensor rightLightSensor = new EV3ColorSensor(SensorPort.S1);
	public static final SampleProvider rightLightProvider = rightLightSensor.getMode("RGB");

	public static final EV3ColorSensor leftLightSensor = new EV3ColorSensor(SensorPort.S4);
	public static final SampleProvider leftLightProvider = leftLightSensor.getMode("RGB");
		
	//Robot related parameters
	public static final double WHEEL_RAD = 2.1;
    public static final double TRACK = 15.68;
    /**
     * This is the distance separating the wheels from both light sensors
     */
	public static final double DIST = 5.27;
	/**
	 * This is the light sensor R reading that corresponds to a black line
	 */
	public static final double BLACK = 0.08;
	public static final int FAST = 170;
	public static final int SLOW = 130;
	public static final double TILE = 30.48;

	/**
	 * Main method where we create our Odometer, CanScanner, Navigation, Localization objects and start the Wifi and Odometer Threads
	 * @param args
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions{
	    
		  // Odometer related objects
		  final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor);

		  // Odometer related objects
		  final CanScanner canScanner = CanScanner.getCanScanner(leftMotor, rightMotor, rotatingArmMotor, gateMotor, rotatingLightProvider, usProvider);
		  
		  // Start odometer thread
	      Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      
	      // Search related objects
	      final Navigation navigation = Navigation.getNavigation(leftMotor, rightMotor, usProvider, canScanner);
	      
	      // UltrasonicLocalization related objects
		  final UltrasonicLocalization usLocalization = new UltrasonicLocalization(leftMotor, rightMotor, usSensor);
		  //LightLocalization related objects
	      final LightLocalization lightLocalization = new LightLocalization(leftMotor, rightMotor, usProvider, leftLightProvider, rightLightProvider, navigation);
	      
	      final Testing test = new Testing(leftMotor,	rightMotor,	rotatingArmMotor, gateMotor, rotatingLightProvider, leftLightProvider, rightLightProvider, usProvider, navigation);
	      
	      // Wifi related objects
	      final Wifi wifi = new Wifi();
	      	      	      
	      //Spawn the main Thread for our project 
	      (new Thread() {
			public void run() {
				Sound.setVolume(100);
			
				wifi.run();
				
				usLocalization.fallingEdge();				
				lightLocalization.localize();
				
				navigation.travelToTunnelEntrance(false);
				lightLocalization.reLocalize();
				odometer.setXYT(Wifi.tunnelEntrance_x*30.48, Wifi.tunnelEntrance_y*30.48, 0);
				navigation.travelToTunnelExit(true);
				
				navigation.travelToSearchCenter();
				
				navigation.searchRoutine2();	
				
				navigation.travelToTunnelExit(false);
				lightLocalization.reLocalize();
				odometer.setXYT(Wifi.tunnelExit_x*30.48, Wifi.tunnelExit_y*30.48, 0);
				navigation.travelToTunnelEntrance(true);

				//Full localization

				//usLocalization.fallingEdge();				
				//lightLocalization.localize();
				
				//Travel to search area				
				//navigation.travelToTunnelEntrance();
				
				//navigation.turnToFace(0);
				//lightLocalization.localize2();
				//navigation.travelToTunnelExit();
				
				//navigation.RegularTravelTo( Wifi.searchZone_LL_x*TILE, Wifi.searchZone_LL_y*TILE);
				
				//Searching 
				//navigation.searchRoutine();
				
				//Travel to starting corner
				//navigation.travelToTunnelExit();
				//navigation.travelToTunnelEntrance();
				//navigation.travelToStartCorner();
				
				for(int i=0 ; i<5 ; i++) {
					Sound.beep();					
					try {
						Thread.sleep(200);
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