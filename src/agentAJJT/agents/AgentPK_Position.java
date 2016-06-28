/**
 * *****************************************************************************
 * RoboNewbie NaoTeam Humboldt
 *
 * @author Monika Domanska
 * @version 1.1
 * *****************************************************************************
 */
package agentAJJT.agents;

import agentAJJT.helper.SmoothParameterArray;
import agentIO.EffectorOutput;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;
import motion.SimplePositionControl;
import util.Logger;
import util.RobotConsts;

/**
 * This agent shows basic concepts of using the RoboNewbie framework and gives
 * examples for interacting with the simulation server and using the classes
 * EffectorOutput and PerceptorInput.
 */
public class AgentPK_Position {

    // TODO: use enum!
    private static final int CONST_STEPHEIGHT = 1;
    private static final int CONST_SHIFT = 2;
    private static final int CONST_STEPLENGTH = 3;
    private static final int CONST_SIDEWARDS = 4;
    private static final int CONST_ALPHA = 5;
    private static final int CONST_OFFSET = 6;

    private final SmoothParameterArray smoothingArray = new SmoothParameterArray(6, 0.1);

    private Logger log;
    private PerceptorInput percIn;
    private EffectorOutput effOut;
    private SimplePositionControl positionControl;

    public static void main(String args[]) {

        // Change here the class to the name of your own agent file 
        // - otherwise Java will always execute the Agent_BasicStructure.
        AgentPK_Position agent = new AgentPK_Position();

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

    /**
     * A player is identified in the server by its player ID and its team name.
     * There are at most two teams an the field, and every agent of a single
     * team must have a unique player ID between 1 and 11. If the identification
     * works right, it is visualized in the monitor: the robots on the field
     * have either red or blue parts. An unidentified robot has grey parts.
     */
    static final String id = "1";
    static final String team = "myT";
    /**
     * The "beam"-coordinates specify the robots initial position on the field.
     * The root of the global field coordinate system is in the middle of the
     * field, the system is right-handed. The x-axis points to the opponent
     * goal, so the initial position must have a negative x-value. The robot can
     * be placed with an initial orientation given in the variable beamRot, in
     * degrees, counterclockwise relative to the x-axis.
     */
    static final double beamX = -1;
    static final double beamY = 0;
    static final double beamRot = 0;

    /**
     * Initialize the connection to the server, the internal used classes and
     * their relations to each other, and create the robot at a specified
     * position on the field.
     */
    private void init() {

        // connection to the server
        ServerCommunication sc = new ServerCommunication();

        // internal agent classes
        log = new Logger();
        percIn = new PerceptorInput(sc);
        effOut = new EffectorOutput(sc);
        positionControl = new SimplePositionControl(percIn, effOut);

        // simulated robot hardware on the soccer field
        sc.initRobot(id, team, beamX, beamY, beamRot);
    }

    /**
     * Main loop of the agent program, where it is synchronized with the
     * simulation server.
     *
     * @param timeInSec Time in seconds the agent program will run.
     */
    private void run(int timeInSec) {
        //Variables to set
        //how much the feet are raised
        smoothingArray.setGoal(CONST_STEPHEIGHT, 10.0);

        //how much the robot shifts its hips from one side to another 
        smoothingArray.setGoal(CONST_SHIFT, 15.0);

        // y value - steplength when robot walks forward
        smoothingArray.setGoal(CONST_STEPLENGTH, 0.0);

        //specifies how big the steps to the side are (x direction)
        smoothingArray.setGoal(CONST_SIDEWARDS, 1.0);

        //angle how much the robot turns
        smoothingArray.setGoal(CONST_ALPHA, 0.0);

        //specifies how much the robot goes down at the beginning
        double offset = 25.0;

        double period_in_sec = 2.0; // 2 results in a result equal to the former factor t = t * 3

        //general variables 
        double degree_y;
        double degree_z;

        //variable to walk forward and backward 
        double degree_x_left;
        double degree_x_right;

        //walk sidewards left right - steplength
        double degree_side_right;
        double degree_side_left;

        //walk in a circle - minus to turn clockwise, positive to turn counterclockwise 
        double degree_turn_x;
        double degree_turn_y;

        // degree in total
        double t;
        double rhp;
        double lhp;
        double rkp = 0.0;
        double lkp = 0.0;
        double lfp;
        double rfp;

        // GO DOWN TO START POS     
        double offset_degree = 0.0;
        double offset_rad = Math.toRadians(offset);
        //get servcer time to start with - this hels robot not to fall over 
        sense();
        // act to stay in loop sync
        act();

        double starttime = percIn.getServerTime();
        while (percIn.getJoint(RobotConsts.RightHipPitch) < Math.toRadians(offset)) {
            sense();
            t = percIn.getServerTime() - starttime;
            offset_degree = Math.toRadians(((1 - Math.cos(2 * t)) / 2) * offset);

            positionControl.setJointPosition(RobotConsts.RightHipPitch, offset_degree);
            positionControl.setJointPosition(RobotConsts.LeftHipPitch, offset_degree);
            positionControl.setJointPosition(RobotConsts.RightKneePitch, -2 * offset_degree);
            positionControl.setJointPosition(RobotConsts.LeftKneePitch, -2 * offset_degree);
            positionControl.setJointPosition(RobotConsts.RightFootPitch, offset_degree);
            positionControl.setJointPosition(RobotConsts.LeftFootPitch, offset_degree);

            positionControl.update();
            act();
        }

        sense();
        act();
        starttime = percIn.getServerTime();
        // Loop synchronized with server.
        while (true) {

            // "Hardware" access to the perceptors (simulated sensors) and processing
            // of the perceptor data. 
            sense();

            double[] values = smoothingArray.updateValues();
            double shift = values[CONST_SHIFT];
            double stepheight = values[CONST_STEPHEIGHT];
            double steplength = values[CONST_STEPLENGTH];
            double step_sidewards = values[CONST_SIDEWARDS];
            double alpha = values[CONST_ALPHA];

            t = (percIn.getServerTime() - starttime);

            double period_in_rad_sec = Math.PI * 2.0 / period_in_sec;
            double total_position = t / period_in_sec;
            double position_in_period = total_position % period_in_sec;

            // transform t to rad_sec timescale. t will be 2 pi after period_in_sec seconds
            t = t * period_in_rad_sec;

            //sinus and cosinus to steer the behaviour of the legs 
            degree_y = Math.toRadians(Math.sin(t) * shift);  //move hip left - right
            degree_z = Math.toRadians(((1 - Math.cos(2 * t)) / 2) * stepheight); //raise feet
            degree_x_left = Math.toRadians(((1 - Math.cos(t)) / 2) * steplength); //move left foot foward
            degree_x_right = Math.toRadians(((1 - Math.cos(t + Math.PI)) / 2) * steplength); //move right foot forward
            degree_turn_x = Math.toRadians(((-1 - Math.cos(t + Math.PI)) / 2) * alpha); //turn clockwise 
            degree_turn_y = Math.toRadians(((-1 - Math.cos(t)) / 2) * Math.abs(alpha)); //turn counterclockwise
            //steps to the left and right  
            degree_side_right = Math.toRadians(Math.cos(t) * step_sidewards); //step right
            degree_side_left = Math.toRadians(Math.cos(t + Math.PI) * step_sidewards); //step left

            //move hips to the left and right and lets robot make steps to the side 
            positionControl.setJointPosition(RobotConsts.RightHipRoll, degree_y + degree_side_right);
            positionControl.setJointPosition(RobotConsts.RightFootRoll, -degree_y - degree_side_right);
            positionControl.setJointPosition(RobotConsts.LeftHipRoll, degree_y + degree_side_left);
            positionControl.setJointPosition(RobotConsts.LeftFootRoll, -degree_y - degree_side_left);

            //angles needed in every cycle added up 
            rhp = degree_x_right + offset_rad;
            lhp = degree_x_left + offset_rad;
            rkp = - 2 * offset_rad;
            lkp = - 2 * offset_rad;
            lfp = -degree_x_left + offset_rad;
            rfp = -degree_x_right + offset_rad;

            //for alpha - walking in a circle 
            if (alpha > 0) {
                rhp += degree_turn_x;
            } else {
                lhp += degree_turn_y;
            }

            // raising the feet and moving forward 
            if (degree_y < 0) {
                rhp += degree_z;
                rkp += -2 * degree_z;
                rfp += degree_z;
            } else {
                lhp += degree_z;
                lkp += -2 * degree_z;
                lfp += degree_z;
            }
            //right leg 
            positionControl.setJointPosition(RobotConsts.RightHipPitch, rhp);
            positionControl.setJointPosition(RobotConsts.RightKneePitch, rkp);
            positionControl.setJointPosition(RobotConsts.RightFootPitch, rfp);
            //left leg 
            positionControl.setJointPosition(RobotConsts.LeftHipPitch, lhp);
            positionControl.setJointPosition(RobotConsts.LeftKneePitch, lkp);
            positionControl.setJointPosition(RobotConsts.LeftFootPitch, lfp);

            //turn robot
            if (alpha > 0) {
                positionControl.setJointPosition(RobotConsts.RightHipYawPitch, degree_turn_x);
            } else {
                positionControl.setJointPosition(RobotConsts.LeftHipYawPitch, degree_turn_y);
            }

            // log.log("reached angle: " + degree
            positionControl.update();
            // "Hardware" access to the effectors (simulated motors).
            act();
        }
    }

    /**
     * Update the world and robot hardware informations, that means process
     * perceptor values provided by the server.
     *
     * Here is listed a simple sequence of method calls, which are executed in
     * every server cycle 1) to synchronize perceptor processing classes with
     * the loop of the simulation server 2) to ensure that the agent gets the
     * actual informations about the robot and the soccer field from the
     * perceptors (simulated sensors).
     */
    private void sense() {
        // Receive the server message and parse it to get the perceptor values. 
        percIn.update();
    }

    /**
     * Move the robot hardware, that means send effector commands to the server.
     *
     * Here is listed a simple sequence of method calls, which are executed in
     * every server cycle 1) to calculate the effector commands, if needed. 2)
     * to send the effector commands to the server regularly in every server
     * cycle.
     *
     * Notice: At least the "syn" effector has to be sent in every server cycle.
     * Look up "agent sync mode" for details.
     */
    private void act() {
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
