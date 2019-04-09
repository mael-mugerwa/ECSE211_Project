package ca.mcgill.ecse211.Project.Wifi;

import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;

/**
 * Example class using WifiConnection to communicate with a server and receive data concerning the
 * competition such as the starting corner the robot is placed in.
 *
 * Keep in mind that this class is an **example** of how to use the WiFi code; you must use the
 * WifiConnection class yourself in your own code as appropriate. In this example, we simply show
 * how to get and process different types of data.
 *
 * There are two variables you **MUST** set manually before trying to use this code.
 *
 * 1. SERVER_IP: The IP address of the computer running the server application. This will be your
 * own laptop, until the beta beta demo or competition where this is the TA or professor's laptop.
 * In that case, set the IP to 192.168.2.3.
 *
 * 2. TEAM_NUMBER: your project team number
 *
 * Note: We System.out.println() instead of LCD printing so that full debug output (e.g. the very
 * long string containing the transmission) can be read on the screen OR a remote console such as
 * the EV3Control program via Bluetooth or WiFi. You can disable printing from the WiFi code via
 * ENABLE_DEBUG_WIFI_PRINT (below).
 *
 * @author Michael Smith, Tharsan Ponnampalam, Mael Mugerwa
 *
 */
public class Wifi implements Runnable{

  // ** Set these as appropriate for your team and current situation **
  //private static final String SERVER_IP = "192.168.2.3";
	
  private static final String SERVER_IP = "192.168.2.42";	
  private static final int TEAM_NUMBER = 13;

  public static int startingCorner;
  
  public static double startZone_LL_x;
  public static double startZone_LL_y;
  public static double startZone_UR_x;
  public static double startZone_UR_y;
  
  public static double tunnel_LL_x;
  public static double tunnel_LL_y;
  public static double tunnel_UR_x;
  public static double tunnel_UR_y;
  
  public static double tunnelEntrance_x;
  public static double tunnelEntrance_y;
  public static double tunnelExit_x;
  public static double tunnelExit_y;
  
  public static boolean tunnelIsHorizontal;
  
  public static double island_LL_x;
  public static double island_LL_y;
  public static double island_UR_x;
  public static double island_UR_y;
  
  public static double searchZone_LL_x;
  public static double searchZone_LL_y;
  public static double searchZone_UR_x;
  public static double searchZone_UR_y;
    
  // Enable/disable printing of debug info from the WiFi class
  private static final boolean ENABLE_DEBUG_WIFI_PRINT = true;

  /**
   * This method correctly initializes the tunnelEntrance point
   */
  public void getTunnelEntrance() {
	  if(startingCorner == 0) {
    	  if(tunnelIsHorizontal) {
    		  tunnelEntrance_x = (tunnel_LL_x-0.5);
    		  tunnelEntrance_y = (tunnel_LL_y+0.5);
    	  }
    	  else {
    		  tunnelEntrance_x = (tunnel_LL_x+0.5);
    		  tunnelEntrance_y = (tunnel_LL_y-0.5);
    	  }
      }
      else if(Wifi.startingCorner == 1) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelEntrance_x = (tunnel_UR_x+0.5);
    		  tunnelEntrance_y = (tunnel_UR_y-0.5);
    	  }
    	  else {
    		  tunnelEntrance_x = (tunnel_LL_x+0.5);
    		  tunnelEntrance_y = (tunnel_LL_y-0.5);
    	  }
      }
      else if(Wifi.startingCorner == 2) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelEntrance_x = (tunnel_UR_x+0.5);
    		  tunnelEntrance_y = (tunnel_UR_y-0.5);
    	  }
    	  else {
    		  tunnelEntrance_x = (tunnel_UR_x-0.5);
    		  tunnelEntrance_y = (tunnel_UR_y+0.5);
    	  }
      }
      else if(Wifi.startingCorner == 3) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelEntrance_x = (tunnel_LL_x-0.5);
    		  tunnelEntrance_y = (tunnel_LL_y+0.5);
    	  }
    	  else {
    		  tunnelEntrance_x  = (tunnel_UR_x-0.5);
    		  tunnelEntrance_y = (tunnel_UR_y+0.5);
    	  }
      }
  }
  
  /**
   * This method correctly initializes the tunnelExit point
   */
  public void getTunnelExit() {
	  if(Wifi.startingCorner == 0) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelExit_x = (tunnel_UR_x+0.5);
    		  tunnelExit_y = (tunnel_UR_y-0.5);
    	  }
    	  else {
    		  tunnelExit_x = (tunnel_UR_x-0.5);
    		  tunnelExit_y = (tunnel_UR_y+0.5);
    	  }
      }
      else if(Wifi.startingCorner == 1) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelExit_x = (tunnel_LL_x-0.5);
    		  tunnelExit_y = (tunnel_LL_y+0.5);
    	  }
    	  else {
    		  tunnelExit_x = (tunnel_UR_x-0.5);
    		  tunnelExit_y = (tunnel_UR_y+0.5);
    	  }
      }
      else if(Wifi.startingCorner == 2) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelExit_x = (tunnel_LL_x-0.5);
    		  tunnelExit_y = (tunnel_LL_y+0.5);  
    	  }
    	  else {
    		  tunnelExit_x = (tunnel_LL_x+0.5);
    		  tunnelExit_y = (tunnel_LL_y-0.5);
    	  }
      }
      else if(Wifi.startingCorner == 3) {
    	  if(Wifi.tunnelIsHorizontal) {
    		  tunnelExit_x = (tunnel_UR_x+0.5);
    		  tunnelExit_y = (tunnel_UR_y-0.5);
    	  }
    	  else {
    		  tunnelExit_x = (tunnel_LL_x+0.5);
    		  tunnelExit_y = (tunnel_LL_y-0.5);
    	  }
      }
  }
  
  @SuppressWarnings("rawtypes")
  public void run(){
    System.out.println("Running..");

    // Initialize WifiConnection class
    WifiConnection conn = new WifiConnection(SERVER_IP, TEAM_NUMBER, ENABLE_DEBUG_WIFI_PRINT);

    // Connect to server and get the data, catching any errors that might occur
    try {
      /*
       * getData() will connect to the server and wait until the user/TA presses the "Start" button
       * in the GUI on their laptop with the data filled in. Once it's waiting, you can kill it by
       * pressing the upper left hand corner button (back/escape) on the EV3. getData() will throw
       * exceptions if it can't connect to the server (e.g. wrong IP address, server not running on
       * laptop, not connected to WiFi router, etc.). It will also throw an exception if it connects
       * but receives corrupted data or a message from the server saying something went wrong. For
       * example, if TEAM_NUMBER is set to 1 above but the server expects teams 17 and 5, this robot
       * will receive a message saying an invalid team number was specified and getData() will throw
       * an exception letting you know.
       */
      Map data = conn.getData();
      
      //Get your team color
      if(((Long) data.get("RedTeam")).intValue() == TEAM_NUMBER) {//your team number corresponds to red team
   
    	//starting corner
    	  startingCorner = ((Long) data.get("RedCorner")).intValue();

    	//startZone LL
    	  startZone_LL_x = ((Long) data.get("Red_LL_x")).doubleValue();
    	  startZone_LL_y = ((Long) data.get("Red_LL_y")).doubleValue();
        //startZone UR
    	  startZone_UR_x = ((Long) data.get("Red_UR_x")).doubleValue();
    	  startZone_UR_y = ((Long) data.get("Red_UR_y")).doubleValue();
    	  
        //tunnel LL
    	  tunnel_LL_x = ((Long) data.get("TNR_LL_x")).doubleValue();
          tunnel_LL_y = ((Long) data.get("TNR_LL_y")).doubleValue();
        //tunnel UR
          tunnel_UR_x = ((Long) data.get("TNR_UR_x")).doubleValue();
          tunnel_UR_y = ((Long) data.get("TNR_UR_y")).doubleValue();
    	//tunnel Orientation
          if(tunnel_UR_y - tunnel_LL_y == 1)
        	  tunnelIsHorizontal = true;
          else if (tunnel_UR_y - tunnel_LL_y == 2)
        	  tunnelIsHorizontal = false;
          
          //tunnel entrance 
          getTunnelEntrance();
          
          //tunnel exit
          getTunnelExit();
          
        //island LL
          island_LL_x = ((Long) data.get("Island_LL_x")).doubleValue();
          island_LL_y = ((Long) data.get("Island_LL_y")).doubleValue();
        //island UR
          island_UR_x = ((Long) data.get("Island_UR_x")).doubleValue();
          island_UR_y = ((Long) data.get("Island_UR_y")).doubleValue();
          
    	//searchZone LL
          searchZone_LL_x = ((Long) data.get("SZR_LL_x")).doubleValue();
          searchZone_LL_y = ((Long) data.get("SZR_LL_y")).doubleValue();
        //searchZone UR
          searchZone_UR_x = ((Long) data.get("SZR_UR_x")).doubleValue();
          searchZone_UR_y = ((Long) data.get("SZR_UR_y")).doubleValue();
                            
      }
      
      else if (((Long) data.get("GreenTeam")).intValue() == TEAM_NUMBER)  {//your team number corresponds to green team
    	//starting corner
    	  startingCorner = ((Long) data.get("GreenCorner")).intValue();

    	//startZone LL
    	  startZone_LL_x = ((Long) data.get("Green_LL_x")).doubleValue();
    	  startZone_LL_y = ((Long) data.get("Green_LL_y")).doubleValue();
        //startZone UR
    	  startZone_UR_x = ((Long) data.get("Green_UR_x")).doubleValue();
    	  startZone_UR_y = ((Long) data.get("Green_UR_y")).doubleValue();
    	  
        //tunnel LL
    	  tunnel_LL_x = ((Long) data.get("TNG_LL_x")).doubleValue();
          tunnel_LL_y = ((Long) data.get("TNG_LL_y")).doubleValue();
        //tunnel UR
          tunnel_LL_x = ((Long) data.get("TNG_UR_x")).doubleValue();
          tunnel_LL_y = ((Long) data.get("TNG_UR_y")).doubleValue();
        //tunnel Orientation
          if(tunnel_UR_y - tunnel_LL_y == 1)
        	  tunnelIsHorizontal = true;
          else if (tunnel_UR_y - tunnel_LL_y == 2)
        	  tunnelIsHorizontal = false;
          
          //tunnel entrance 
          getTunnelEntrance();
          
          //tunnel exit
          getTunnelExit();
          
        //island LL
          island_LL_x = ((Long) data.get("Island_LL_x")).doubleValue();
          island_LL_y = ((Long) data.get("Island_LL_y")).doubleValue();
        //island UR
          island_UR_x = ((Long) data.get("Island_UR_x")).doubleValue();
          island_UR_y = ((Long) data.get("Island_UR_y")).doubleValue();
          
    	//searchZone LL
          searchZone_LL_x = ((Long) data.get("SZG_LL_x")).doubleValue();
          searchZone_LL_y = ((Long) data.get("SZG_LL_y")).doubleValue();
        //searchZone UR
          searchZone_UR_x = ((Long) data.get("SZG_UR_x")).doubleValue();
          searchZone_UR_y = ((Long) data.get("SZG_UR_y")).doubleValue();
      }
      
      else {//error
          System.out.println("Wrong RedTeam and GreenTeam Paramaters");
      }
     
    } catch (Exception e) {
      System.err.println("Error: " + e.getMessage());
    }

  }
}