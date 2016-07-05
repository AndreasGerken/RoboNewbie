/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SmoothParameterArray {

    private List<SmoothParameter> parameters;
    private double standardAlpha;

    /**
     * Specific constructor with all details
     *
     * @param initialValues the values to start the algorithm with
     * @param goals the goal values which should be reached
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(double[] initialValues, double[] goals, double[] alpha) {
        this.parameters = new ArrayList<>();

        for(int i=0; i< initialValues.length; i++){
            parameters.add(new SmoothParameter(initialValues[i], goals[i], alpha[i]));
        }
    }

    /**
     * Generic constructor
     *
     * @param valueCount how many values are needed
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(int valueCount, double alpha) {
        this.parameters = new ArrayList<>();

        for(int i=0; i < valueCount; i++){
            parameters.add(new SmoothParameter(0, 0, alpha));
        }
    }
    
    public SmoothParameterArray(double alpha){
        this.parameters = new ArrayList<>();
        this.standardAlpha = alpha;
    }

    public void setValue(int index, double value) {
        this.parameters.get(index).setValue(value);
    }

    /**
     * Setter for new goals
     *
     * @param newGoals new goal array
     */
    public void setGoals(double[] newGoals) {
        for(int i=0; i < newGoals.length; i++){
            this.parameters.get(i).setGoal(newGoals[i]);
        }
    }

    /**
     * Setter for new goal
     *
     * @param index index in array
     * @param goal value
     */
    public void setGoal(int index, double goal) {
        this.parameters.get(index).setGoal(goal);
    }

    /**
     * updates all values in direction to the goals
     *
     * @return new value array
     */
    public void updateValues() {
        for(SmoothParameter parameter: parameters){
            parameter.updateValue();
        }
    }

    /**
     * Getter for the values
     *
     * @return value array
     */
    public double getValue(int index) {
        return parameters.get(index).getValue();
    }
    
    public SmoothParameter getParameter(int index){
        return parameters.get(index);
    }
    
    public SmoothParameter getNewParameter(){
        // create new Parameter and add it to the list
        SmoothParameter newParameter = new SmoothParameter(0, 0, standardAlpha);
        parameters.add(newParameter);
        
        return newParameter;
    }
}
