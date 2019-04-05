import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

public class Project{
	
	// Motor Objects, Sensor Objects and Robot related parameters
	public static final EV3LargeRegulatedMotor leftMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));
	
	public static final EV3LargeRegulatedMotor rightMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	
	
	public static final EV3MediumRegulatedMotor rotatingArmMotor =
			new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	
	public static final EV3LargeRegulatedMotor gateMotor =
			new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(SensorPort.S2);
	public static final SampleProvider usSensorProvider= usSensor.getMode("Distance");
	
	public static final EV3ColorSensor rotatingLightSensor = new EV3ColorSensor(SensorPort.S3);
	public static final SampleProvider rotatingLightSensorProvider = rotatingLightSensor.getMode("RGB");

	public static final EV3ColorSensor rightLightSensor = new EV3ColorSensor(SensorPort.S1);
	public static final SampleProvider rightLightSensorProvider = rightLightSensor.getMode("RGB");

	public static final EV3ColorSensor leftLightSensor = new EV3ColorSensor(SensorPort.S4);
	public static final SampleProvider leftLightSensorProvider = leftLightSensor.getMode("RGB");
		
	public static final double WHEEL_RAD = 2.1;
    public static final double TRACK =15.59;
	public static final double DIST = 17.35;

	public static final int FAST = 170;
	public static final int SLOW = 130;

	public static final double TILE = 30.48;

	 
	public static void main(String[] args) throws OdometerExceptions{
	    
		  // Odometer related objects
		  final Odometer odometer = Odometer.getOdometer(leftMotor, rightMotor);

		  // Odometer related objects
		  final CanScanner canScanner = CanScanner.getCanScanner(leftMotor, rightMotor, rotatingArmMotor, gateMotor, rotatingLightSensorProvider, usSensorProvider);
		  
		  // Start odometer thread
	      Thread odoThread = new Thread(odometer);
	      odoThread.start();
	      
	      // Search related objects
	      final FinalNavigation navigation = FinalNavigation.getNavigation(leftMotor, rightMotor, usSensorProvider, canScanner,leftLightSensor, rightLightSensor);
	      
	      // Localization related objects
		  
	      final UltrasonicLocalizer usLocalization = new UltrasonicLocalizer(leftMotor, rightMotor, usSensor);
		  //final US us = new US( odometer,  leftMotor,  rightMotor, usSensorProvider);
	      //final LightLocalizer lightLocalization = new LightLocalizer(leftMotor, rightMotor, rightLightSensor);   	   		
	      final FullLocalization fullLocl = new FullLocalization(leftMotor, rightMotor, usSensorProvider, leftLightSensorProvider, rightLightSensorProvider);

	      // Wifi related objects
	      final Wifi wifi = new Wifi();
	      
	      // Start wifi thread
	      Thread wifiThread = new Thread(wifi);
	      wifiThread.start();
	      	      
	      //spawn a new Thread for project 
	      (new Thread() {
			public void run() {
				
				
				Sound.setVolume(100);	
	      		
//Full localization
				//us.localizeFallingEdge();
				usLocalization.fallingEdge();				
			//	navigation.t
//			
				//navigation.travelToSearchArea();
				
			//	navigation.searchRoutine();
	        }
	      }).start();	       
	      
	      while (Button.waitForAnyPress() != Button.ID_ESCAPE);
	      System.exit(0);
	}
	  
}