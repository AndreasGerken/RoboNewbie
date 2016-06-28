/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

import java.util.Arrays;

public class SmoothParameterArray {

    private SmoothParameter[] parameters;

    /**
     * Specific constructor with all details
     *
     * @param initialValues the values to start the algorithm with
     * @param goals the goal values which should be reached
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(double[] initialValues, double[] goals, double[] alpha) {
        this.parameters = new SmoothParameter[initialValues.length];

        for(int i=0; i<parameters.length; i++){
            parameters[i] = new SmoothParameter(initialValues[i], goals[i], alpha[i]);
        }
    }

    /**
     * Generic constructor
     *
     * @param valueCount how many values are needed
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(int valueCount, double alpha) {
        this.parameters = new SmoothParameter[valueCount];

        for(int i=0; i<parameters.length; i++){
            parameters[i] = new SmoothParameter(0, 0, alpha);
        }
    }

    public void setValue(int index, double value) {
        this.parameters[index].setValue(value);
    }

    /**
     * Setter for new goals
     *
     * @param newGoals new goal array
     */
    public void setGoals(double[] newGoals) {
        for(int i=0; i<parameters.length; i++){
            this.parameters[i].setGoal(newGoals[i]);
        }
    }

    /**
     * Setter for new goal
     *
     * @param index index in array
     * @param goal value
     */
    public void setGoal(int index, double goal) {
        this.parameters[index].setGoal(goal);
    }

    /**
     * updates all values in direction to the goals
     *
     * @return new value array
     */
    public void updateValues() {
        for(int i=0; i<parameters.length; i++){
            parameters[i].updateValue();
        }
    }

    /**
     * Getter for the values
     *
     * @return value array
     */
    public double getValue(int index) {
        return parameters[index].getValue();
    }
    
    public SmoothParameter getParameter(int index){
        return parameters[index];
    }
}
