package agentAJJT.agents;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import motion.SimplePositionControl;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.Logger;
import util.RobotConsts;

/**
 * This agent shows basic concepts of using the RoboNewbie framework and gives
 * examples for interacting with the simulation server and using the classes
 * EffectorOutput and PerceptorInput.
 */
public class OrientationPerceptorTestAgent {

  public static void main(String args[]) {
    
    // Change here the class to the name of your own agent file 
    // - otherwise Java will always execute the Agent_BasicStructure.
    OrientationPerceptorTestAgent agent = new OrientationPerceptorTestAgent();
    
    // Establish the connection to the server.
    agent.init();
    
    // Run the agent program synchronized with the server cycle.
    // Parameter: Time in seconds the agent program will run. 
    agent.run(12);

    // The logged informations are printed here, when the agent is not timed 
    // with the server anymore. Printing immediately when informations are 
    // gained during the server cycles could slow down the agent and impede 
    // the synchronization.
    agent.printlog();
    
    System.out.println("Agent stopped.");
  }

  private Logger log;
  private PerceptorInput percIn;
  private OrientationPerceptor op;
  private EffectorOutput effOut;
  private SimplePositionControl positionControl; 

  /** A player is identified in the server by its player ID and its team name. 
   There are at most two teams an the field, and every agent of a single team 
   must have a unique player ID between 1 and 11. 
   If the identification works right, it is visualized in the monitor: 
   the robots on the field have either red or blue parts. An unidentified 
   robot has grey parts. */
  static final String id = "1";
  static final String team = "myT";
  /** The "beam"-coordinates specify the robots initial position on the field.
   The root of the global field coordinate system is in the middle of the 
   field, the system is right-handed. The x-axis points to the opponent goal, 
   so the initial position must have a negative x-value. The robot can be placed
   with an initial orientation given in the variable beamRot, in degrees, 
   counterclockwise relative to the x-axis. */
  static final double beamX =    -1;
  static final double beamY =     0;
  static final double beamRot =   0;
    
  /**
   * Initialize the connection to the server, the internal used classes and 
   * their relations to each other, and create the robot at a specified position 
   * on the field. 
   */
  private void init() {

    // connection to the server
    ServerCommunication sc = new ServerCommunication();

    // internal agent classes
    log = new Logger();
    percIn = new PerceptorInput(sc);
        
    op = new OrientationPerceptor(percIn);
            
    effOut = new EffectorOutput(sc);
    positionControl = new SimplePositionControl(percIn, effOut);
    
    
    // simulated robot hardware on the soccer field
    sc.initRobot(id, team, beamX, beamY, beamRot);
  }

  private double scale(double min, double max, double y) {
      double amplitude = Math.toRadians(max - min);
      double offset = Math.toRadians(min) + 0.5 * amplitude;
      return y * 0.5*amplitude + offset;
  }  
  
  /**
   * Main loop of the agent program, where it is synchronized with the 
   * simulation server. 
   * 
   * @param timeInSec Time in seconds the agent program will run. 
   */
    private void run(int timeInSec) {             
        
        double t = 0;
        //double alpha = 0.5;        
        
        // Loop synchronized with server.
        //while (true) {
        for (int i = 0; i < 100; i++) {

            // "Hardware" access to the perceptors (simulated sensors) and processing
            // of the perceptor data. 
            sense();

            // Accelerometer and gyro rate perceptors.
            // They are both given as Vector3D instances, so they are accessed in the 
            // same way.
            Vector3D acc = percIn.getAcc();
            log.log("Acc test output - " + Logger.cartesianStr(acc));
            //log.log("Acc value access - value in z-direction: " + acc.getZ());

            Vector3D gyro = percIn.getGyro();
            log.log("Gyro test output - " + Logger.cartesianStr(gyro));

            log.log("Orientation test output - " + Logger.cartesianStr(op.getOrientation(0.5)));
            
            
            t = percIn.getServerTime();
            
            
            double hip_pitch = scale(30, 0, Math.sin(t)) * 1.0;
            
            positionControl.setJointPosition(RobotConsts.LeftHipPitch, hip_pitch);
            positionControl.setJointPosition(RobotConsts.RightHipPitch, hip_pitch);
                 
            positionControl.update();            
            
            act();
        }
    }

  /**
   * Update the world and robot hardware informations, that means process 
   * perceptor values provided by the server.
   * 
   * Here is listed a simple sequence of method calls, which are executed in 
   * every server cycle 
   * 1) to synchronize perceptor processing classes with the loop of the simulation 
   * server 
   * 2) to ensure that the agent gets the actual informations about the robot and 
   * the soccer field from the perceptors (simulated sensors).
   */
  private void sense() {
    // Receive the server message and parse it to get the perceptor values. 
    percIn.update();
  }
  
  /**
   * Move the robot hardware, that means send effector commands to the server. 
   * 
   * Here is listed a simple sequence of method calls, which are executed in 
   * every server cycle 
   * 1) to calculate the effector commands, if needed.
   * 2) to send the effector commands to the server regularly in every server 
   * cycle. 
   * 
   * Notice: At least the "syn" effector has to be sent in every server cycle.
   * Look up "agent sync mode" for details.
   */
  private void act(){
    // Send agent message with effector commands to the server.
    effOut.sendAgentMessage();
  }

  /**
   * Print logged informations. 
   */
  private void printlog() {
    log.printLog();
  }
  
}
