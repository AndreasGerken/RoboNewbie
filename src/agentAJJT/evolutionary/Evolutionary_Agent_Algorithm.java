/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package agentAJJT.evolutionary;

import agentAJJT.agents.AgentPK_Velocity;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 *
 * @author Gerken
 */
public class Evolutionary_Agent_Algorithm {
    public static void main(String args[]){
        List<Gene> genePool = new ArrayList<>();
        Gene bestGene = null;

        
            double hip_factor = 2.0;
            double arm_factor = 3.0;
            double leg_raise_factor = 5.0;
            double leg_push_factor = 0.8;
            double period_in_sec = 2.5;
            double motorspeed = 0.1;

            double startVariables[] = {hip_factor, arm_factor, leg_raise_factor, leg_push_factor, period_in_sec, motorspeed};
            double startStepwidth[] = new double[startVariables.length];
            for(int i = 0; i< startStepwidth.length; i++){
                startStepwidth[i] = startVariables[i] * 0.0001;
            }


            //double startVariables[] = {2.0042178198285927, 2.978934073419497, 5.097006721749297, 0.7998114090729795, 2.501298578822571, 0.10227419648127235};
            //double startStepwidth[] = {0.00853429222898684, 5.878745301724463E-4, 6.721628638347231E-4, 3.648498197218623E-11, 0.0013989369696712857, 4.5654040146536997E-4};
            
            //double startVariables[] = {1.998666241002734, 2.985859376971036, 5.00070973342586, 0.8755212626324911, 2.24544749894646, 0.0996340587101301};
            //double startStepwidth[] = {7.002556960324526E-5, 2.176145306797385E-5, 6.129475497823641E-5, 1.6574100530246336E-5, 0.009843435909475798, 8.157304816334376E-5};
            
            double startStepwidthChange = 1.3f;
            
            Gene root = null;
            
        try{        
            Random rnd = new Random();
            root = new Gene(startVariables, startStepwidth, startStepwidthChange, rnd);
            
            //for(int i = 0; i< startStepwidth.length; i++){
            //    startStepwidth[i] = startVariables[i] * 0.0001;
            //}

            int generations = 100;
            int individuals = 100;
            int individuals_to_double_test = 35;
            int individualsToLive = 20;

            AgentPK_Velocity agent = new AgentPK_Velocity();
            agent.init();

            
            testAgent(root, agent,3);
            bestGene = root;
            System.out.println("Root fitness: " + root.getFitness());

            for(int individual = 0; individual < individuals; individual++){
                genePool.add(root.createChild());
            }        


            for(int generation = 0; generation < generations; generation++){
                System.out.println("Generation " + generation);

                int gCounter = 1;
                for(Gene gene: genePool){
                    System.out.println("Gene no " + (gCounter++) +" of "+genePool.size());
                    testAgent(gene, agent,1);
                    System.out.println(gene.getFitness());
                    
                }
                Collections.sort(genePool);
                genePool = genePool.subList(0,individuals_to_double_test);
                
                gCounter = 1;
                for(Gene gene: genePool){
                    System.out.println("Refine no " + (gCounter++) +" of "+genePool.size());
                    System.out.println("Firstresult" + gene.getFitness());
                    testAgent(gene,agent,2);
                    System.out.println("Doubletest" + gene.getFitness());
                }
                Collections.sort(genePool);
                genePool = genePool.subList(0,individualsToLive);
                
                if(genePool.get(0).getFitness() < bestGene.getFitness()){
                    System.out.println("BETTER! by " + (bestGene.getFitness() - genePool.get(0).getFitness()));
                    bestGene = genePool.get(0);
                }
                
                genePool.add(bestGene);

                System.out.println("Best genes:");
                for(Gene gene:genePool){
                    System.out.println("\t" + gene.getFitness());
                }

                System.out.print("bestVar: "+bestGene.getFitness() + "\n");
                System.out.print("double startVariables[] = {");
                for(int i = 0; i<bestGene.variables.length; i++){
                    if(i != 0){
                        System.out.print(", ");
                    }
                    System.out.print(bestGene.variables[i]);
                }
                System.out.print("};\n");

                System.out.print("double startStepwidth[] = {");
                for(int i = 0; i<bestGene.variables.length; i++){
                    if(i != 0){
                        System.out.print(", ");
                    }
                    System.out.print(bestGene.stepwidth[i]);
                }
                System.out.print("};\n");
                

                List<Gene> newGenePool = new ArrayList<>();
                for(int i = 0; i<individuals; i++){
                    Gene selectedGene = genePool.get(rnd.nextInt(individualsToLive));
                    newGenePool.add(selectedGene.createChild());
                }
                genePool = newGenePool;
            }

            System.out.println("Var: " +bestGene.variables);
            System.out.println("Step: " + bestGene.stepwidth);
            System.out.println("Fit: " + bestGene.getFitness());
        }catch(Exception e){
            
            if(bestGene == root){
                System.out.println("THIS IS ROOOOT!");
            }
            
            System.out.print("bestVar: "+bestGene.getFitness() + "\n");
            System.out.print("double startVariables[] = {");
            for(int i = 0; i<bestGene.variables.length; i++){
                if(i != 0){
                    System.out.print(", ");
                }
                System.out.print(bestGene.variables[i]);
            }
            System.out.print("};\n");

            System.out.print("double startStepwidth[] = {");
            for(int i = 0; i<bestGene.variables.length; i++){
                if(i != 0){
                    System.out.print(", ");
                }
                System.out.print(bestGene.stepwidth[i]);
            }
            System.out.print("};\n");
            e.printStackTrace();
        }
        
    }
    
    static public float testAgent(Gene gene, Evolutionary_Agent agent, int repeat){
        for(int i=0; i< repeat; i++){

            agent.setVariables(gene.variables);
            agent.run(800);
            gene.addFitness(agent.getFitness());

            //agent = new Agent_parallel_controll();
            agent.resetJoints();
            agent.beamMe();
        }

        return 0.0f;
        
    }

}

