/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package agentAJJT.helper;

public class SmoothParameter {

    double alpha;
    double savedValue;

    public SmoothParameter(double alpha) {
        this.alpha = alpha;
    }

    public double newValue(double value) {
        savedValue = (1 - alpha) * savedValue + alpha * value;
        return savedValue;
    }
}
