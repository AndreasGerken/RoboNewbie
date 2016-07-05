/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

import java.util.ArrayList;
import java.util.List;

public class SmoothParameterArray {

    private final List<SmoothParameter> parameters;
    private double standardAlpha;

    /**
     * Specific constructor with all details. The SmoothParameters will be
     * created and should be gathered with getParameter(i)
     *
     * @param initialValues the values to start the algorithm with
     * @param goals the goal values which should be reached
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(double[] initialValues, double[] goals, double[] alpha) {
        this.parameters = new ArrayList<>();

        for (int i = 0; i < initialValues.length; i++) {
            parameters.add(new SmoothParameter(initialValues[i], goals[i], alpha[i]));
        }
    }

    /**
     * Constructor which creates SmoothParameters with no initialization but the
     * alpha value
     *
     * @param valueCount how many values are needed
     * @param alpha the speed of the smoothing (1.0 is imediate 0.001 is slow)
     */
    public SmoothParameterArray(int valueCount, double alpha) {
        this.standardAlpha = alpha;
        this.parameters = new ArrayList<>();

        for (int i = 0; i < valueCount; i++) {
            parameters.add(new SmoothParameter(0, 0, this.standardAlpha));
        }
    }

    /**
     * Constructor which creates just the empty object. Parameters should be
     * created with getNewParameter.
     *
     * @param alpha standard alpha which will be used for all following
     * parameters
     */
    public SmoothParameterArray(double alpha) {
        this.parameters = new ArrayList<>();
        this.standardAlpha = alpha;
    }

    /**
     * Constructor which creates just the empty object.
     */
    public SmoothParameterArray() {
        this.parameters = new ArrayList<>();
    }

    /**
     * Creates a new Parameter with the standard alpha and gives it back
     *
     * @return new Parameter
     */
    public SmoothParameter getNewParameter() {
        // create new Parameter and add it to the list
        SmoothParameter newParameter = new SmoothParameter(0, 0, standardAlpha);
        parameters.add(newParameter);

        return newParameter;
    }

    /**
     * Setter for one value
     *
     * @param index index of the parameter
     * @param value new value
     */
    public void setValue(int index, double value) {
        this.parameters.get(index).setValue(value);
    }

    /**
     * Setter for new goals
     *
     * @param newGoals new goal array
     */
    public void setGoals(double[] newGoals) {
        for (int i = 0; i < newGoals.length; i++) {
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
     * Setter for alphas for all parameters. New Parameters will also be created
     * with this alpha (standard alpha).
     *
     * @param alpha alpha value for all old and new parameters
     */
    public void setAllAlphas(double alpha) {
        for (SmoothParameter parameter : parameters) {
            parameter.setAlpha(alpha);
        }

        this.standardAlpha = alpha;
    }

    /**
     * updates all values in direction to the goals
     */
    public void updateValues() {
        for (SmoothParameter parameter : parameters) {
            parameter.updateValue();
        }
    }

    /**
     * Getter for one value
     *
     * @param index index in array
     * @return value at given index
     */
    public double getValue(int index) {
        return parameters.get(index).getValue();
    }

    /**
     * Getter for Parameters. The parameters have to be created before
     *
     * @param index index in array
     * @return SmoothParameter object at given index
     */
    public SmoothParameter getParameter(int index) {
        return parameters.get(index);
    }
}
