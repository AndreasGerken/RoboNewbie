/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package agentAJJT.evolutionary;

import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import motion.SimplePositionControl;
import util.Logger;
import util.RobotConsts;


abstract public class Evolutionary_Agent {
    protected ServerCommunication sc;
    protected Logger log;
    protected PerceptorInput percIn;
    protected EffectorOutput effOut;
    
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
    static final double beamY =     -2;
    static final double beamRot =   -180;
    
    public void init(){
        // connection to the server
        sc = new ServerCommunication();

        // internal agent classes
        log = new Logger();
        percIn = new PerceptorInput(sc);
        effOut = new EffectorOutput(sc);

        sc.initRobot(id, team, beamX, beamY, beamRot);
    }
    
    abstract public void setVariables(double[] variables);
    abstract public void run(int cycles2run);
    abstract public double getFitness();
    abstract public void reset();
    
    public void beamMe(){
        // beam back to begin
        sc.sendBeamMessage();
    }
      
    public void resetJoints(){
        int cycles2reset = 50;
        double[] neutralPosition = new double[RobotConsts.JointsCount];

        for(int i=0; i < neutralPosition.length; i++){
            neutralPosition[i] = 0;
        }

        SimplePositionControl positionControl = new SimplePositionControl(percIn, effOut);
        positionControl.setAllJointPositions(neutralPosition);
        //double degree;

        // do nothing for 2 seconds, just stay synchronized with the server
        for (int i = 0; i < cycles2reset; i++){
            sense();
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
  protected void sense() {
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
  protected void act(){
    // Send agent message with effector commands to the server.
    effOut.sendAgentMessage();
  }
}
