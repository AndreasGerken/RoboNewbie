/*******************************************************************************
*  RoboNewbie
* NaoTeam Humboldt
* @author Monika Domanska
* @version 1.1
*******************************************************************************/

package agentAJJT.agents;

import agentAJJT.evolutionary.*;
import util.RobotConsts;
import motion.SimplePositionControl; 
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import util.FieldConsts;

/**
 * This agent shows basic concepts of using the RoboNewbie framework and gives
 * examples for interacting with the simulation server and using the classes
 * EffectorOutput and PerceptorInput.
 */
public class AgentPK_Velocity extends Evolutionary_Agent{
  double hip_factor, arm_factor, leg_raise_factor, leg_push_factor, period_in_sec, motorspeed;

  Vector3D g1l, g2l;
  boolean fallen = false;
  boolean syncing = false;
  
  public static void main(String args[]) {
    
    // Change here the class to the name of your own agent file 
    // - otherwise Java will always execute the Agent_BasicStructure.
    AgentPK_Velocity agent = new AgentPK_Velocity();
    
    // Establish the connection to the server.
    // MAGIC NUMBERS :) this controlls the how fast the joints move and so how far they move (with fixed period)
    //this.hip_factor = variables[0];
    //this.arm_factor = variables[1];
    //this.leg_raise_factor = variables[2];
    //this.leg_push_factor = variables[3];
    //this.period_in_sec = variables[4];
    //this.motorspeed = variables[5];
    double hip_factor = 2.0;
    double arm_factor = 3.0;
    double leg_raise_factor = 5.0;
    double leg_push_factor = 0.8;
    double period_in_sec = 2.5;
    double motorspeed = 0.1;
    
    double variables[] = {hip_factor, arm_factor, leg_raise_factor, leg_push_factor, period_in_sec, motorspeed};
    
    
    agent.init();
    agent.setVariables(variables);
    
    // Run the agent program synchronized with the server cycle.
    // Parameter: Time in seconds the agent program will run. 
    agent.run(1000);

    // The logged informations are printed here, when the agent is not timed 
    // with the server anymore. Printing immediately when informations are 
    // gained during the server cycles could slow down the agent and impede 
    // the synchronization.
    agent.printlog();
    
    System.out.println("Agent stopped.");
  }

    
  /**
   * Initialize the connection to the server, the internal used classes and 
   * their relations to each other, and create the robot at a specified position 
   * on the field. 
   */
  @Override
  public void setVariables(double[] variables) {
    this.hip_factor = variables[0];
    this.arm_factor = variables[1];
    this.leg_raise_factor = variables[2];
    this.leg_push_factor = variables[3];
    this.period_in_sec = variables[4];
    this.motorspeed = variables[5];
  }
    
  /**
   * Main loop of the agent program, where it is synchronized with the 
   * simulation server. 
   * 
   * @param cycles2run Time in seconds the agent program will run. 
   */
  public void run(int cycles2run) {
    // The server executes about 50 cycles per second. 
    //int cycles = timeInSec * 50;
    int cycles2sec = 150;
    //double degree;
    
    // do nothing for 2 seconds, just stay synchronized with the server
    syncing = true;
    for (int i = 0; i < cycles2sec; i++){
      sense();
      act();
    }
    syncing = false;
    
    this.reset();
    
    
    // Loop synchronized with server.
    
    double startTime = percIn.getServerTime();
    
    for(int cycle = 0; cycle < cycles2run; cycle++){
      
        if(fallen){
            System.out.println("Fallen");
            break;
        }
      
      // "Hardware" access to the perceptors (simulated sensors) and processing
      // of the perceptor data. 
      sense();
      
      
      // "Think":
      // Use the perceptor data (simulated sensory data, here gained by percIn) 
      // to control the effectors (simulated motors, here activated by effOut) 
      // accordingly.
      // This agent raises the robots arms to approximately 30°.
      //degree = Math.toDegrees(percIn.getJoint(RobotConsts.LeftShoulderPitch));
      //if (degree < 30) {
      //  effOut.setJointCommand(RobotConsts.LeftShoulderPitch,  1.0);
      //  effOut.setJointCommand(RobotConsts.RightShoulderPitch, 1.0);
      //} else {
      //  effOut.setJointCommand(RobotConsts.LeftShoulderPitch, 0.0);
      //  effOut.setJointCommand(RobotConsts.RightShoulderPitch, 0.0);
      //}
      
      //log.log("reached angle: " + degree);
      
      double time = percIn.getServerTime() - startTime;
      double t = time;
      
      double period_in_rad_sec = Math.PI * 2.0 / period_in_sec;
      double total_position = t/period_in_sec;
      double position_in_period = total_position % period_in_sec;
      
      double fsin = Math.sin(t * period_in_rad_sec);
      double fsin3 = Math.pow(fsin,3);
      double fcos = Math.cos(t * period_in_rad_sec);
      
      // fast sin is double the frequency and to the power of 5
      // this means that a leg can be raised fast (power of 5) and lowered fast in the time, the time where the period is on the other side
      double fast_sin5 = Math.pow(Math.sin(t * period_in_rad_sec * 2.0),5);  //(faster)
      
      double motorspeed_in_period = motorspeed * period_in_sec;
      

      
      
      // move the hip from left to right
      effOut.setJointCommand(RobotConsts.LeftHipRoll, -fcos * motorspeed_in_period * hip_factor);
      effOut.setJointCommand(RobotConsts.RightHipRoll, -fcos * motorspeed_in_period * hip_factor);
      effOut.setJointCommand(RobotConsts.LeftFootRoll, fcos * motorspeed_in_period * hip_factor);
      effOut.setJointCommand(RobotConsts.RightFootRoll, fcos * motorspeed_in_period * hip_factor);
      
      // save hip and knee pitch to add values later
      // left, right hip pitch
      double rhp = 0.0, lhp = 0.0;
      // left, right knee pitch
      double lkp = 0.0, rkp = 0.0;
      
      // if position in period is smaller than half controll right leg else left leg
      if(position_in_period < 0.5){
        // raise right leg
        rhp = fast_sin5 * motorspeed_in_period * leg_raise_factor;
        rkp = -fast_sin5 * 2.0 * motorspeed_in_period * leg_raise_factor;
        effOut.setJointCommand(RobotConsts.RightFootPitch, fast_sin5 * motorspeed_in_period * leg_raise_factor);
      }else{
        // raise left leg
        lhp = fast_sin5 * motorspeed_in_period * leg_raise_factor;
        lkp = -fast_sin5 * 2.0 * motorspeed_in_period * leg_raise_factor;
        effOut.setJointCommand(RobotConsts.LeftFootPitch, fast_sin5 * motorspeed_in_period * leg_raise_factor);
      }
      
      // move right leg to front and back
      rhp += fsin3 * motorspeed_in_period * leg_push_factor;
      rkp += -fsin3 * motorspeed_in_period * leg_push_factor;
      
      // start a half period later with the left leg
      if(total_position > 0.5){
        lhp -= fsin3 * motorspeed_in_period * leg_push_factor;
        lkp -= -fsin3 * motorspeed_in_period * leg_push_factor;
      }
      
      // send actor values
      effOut.setJointCommand(RobotConsts.RightHipPitch, rhp);
      effOut.setJointCommand(RobotConsts.RightKneePitch, rkp);
      effOut.setJointCommand(RobotConsts.LeftHipPitch, lhp);
      effOut.setJointCommand(RobotConsts.LeftKneePitch, lkp);
      
      // move arms to mimic zombie
      effOut.setJointCommand(RobotConsts.LeftShoulderPitch, -fcos * motorspeed_in_period * arm_factor);
      effOut.setJointCommand(RobotConsts.RightShoulderPitch, fcos * motorspeed_in_period * arm_factor);
        
      // "Hardware" access to the effectors (simulated motors).
      act();
    }
  }
  

  
  @Override
  public double getFitness(){
      try{
        double norm = g1l.getNorm() + g2l.getNorm();
        if(fallen){
            return norm + 10.0;
        }
        return norm;
      }catch(Exception e){
          return 100.0;
      }
  }
  
  @Override
  protected void sense() {
    super.sense();
    
    if(!fallen && !syncing){
        if((double)percIn.getAcc().getZ() > 7.0){
            Vector3D ng1l = percIn.getGoalPost(FieldConsts.GoalPostID.G1L);
            Vector3D ng2l = percIn.getGoalPost(FieldConsts.GoalPostID.G2L);
            
            if(ng1l != null){
                g1l = ng1l;
            }
            if(ng2l != null){
                g2l = ng2l;
            }

        }else{
            fallen = true;
        }
    }
  }
      
  /**
   * Print logged informations. 
   */
  private void printlog() {
    log.printLog();
  }  

    @Override
    public void reset() {
        fallen = false;
        g1l = null;
        g2l = null;
    }
}
