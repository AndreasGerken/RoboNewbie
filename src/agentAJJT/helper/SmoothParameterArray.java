/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

import java.util.Arrays;

public class SmoothParameterArray {

    private double[] alpha;
    private double[] savedValues;
    private double[] goals;

    /**
     * Specific constructor with all details
     *
     * @param initialValues the values to start the algorithm with
     * @param goals the goal values which should be reached
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(double[] initialValues, double[] goals, double[] alpha) {
        this.savedValues = initialValues;
        this.goals = goals;
        this.alpha = alpha;
    }

    /**
     * Generic constructor
     *
     * @param valueCount how many values are needed
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(int valueCount, double alpha) {
        this.savedValues = new double[valueCount];
        this.goals = new double[valueCount];
        this.alpha = new double[valueCount];
        Arrays.fill(this.alpha, alpha);
    }

    public void setValue(int index, double value) {
        this.savedValues[index] = value;
    }

    /**
     * Setter for new goals
     *
     * @param newGoals new goal array
     */
    public void setGoals(double[] newGoals) {
        this.goals = newGoals;
    }

    /**
     * Setter for new goal
     *
     * @param index index in array
     * @param goal value
     */
    public void setGoal(int index, double goal) {
        this.goals[index] = goal;
    }

    /**
     * updates all values in direction to the goals
     *
     * @return new value array
     */
    public double[] updateValues() {
        for (int i = 0; i < savedValues.length; i++) {
            savedValues[i] = (1 - alpha[i]) * savedValues[i] + alpha[i] * goals[i];
        }

        return savedValues;
    }

    /**
     * Getter for the values
     *
     * @return value array
     */
    public double[] getValues() {
        return savedValues;
    }
}
