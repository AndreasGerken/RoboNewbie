/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

public class SmoothParameter {

    double alpha;
    double savedValue;
    double goal;

    public SmoothParameter(double initialValue, double goal, double alpha) {
        this.savedValue = initialValue;
        this.goal = goal;
        this.alpha = alpha;
    }

    public void setValue(double value) {
        this.savedValue = value;
    }

    public double getValue() {
        return savedValue;
    }

    public void setGoal(double goal) {
        this.goal = goal;
    }

    public double updateValue() {
        savedValue = (1 - alpha) * savedValue + alpha * goal;
        return savedValue;
    }

    public void setAlpha(double alpha) {
        this.alpha = alpha;
    }
}
