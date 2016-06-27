package agentAJJT.evolutionary;


import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

public class Gene implements Comparable<Gene> {
    double[] variables;
    double[] stepwidth;
    double stepwidthChange;
    private List<Double> fitness;
    Random rnd;

    public Gene(double[] variables, double[] stepwidth, double stepwidthChange, Random rnd){
        this.variables = variables;
        this.stepwidth = stepwidth;
        this.stepwidthChange = stepwidthChange;
        this.fitness = new ArrayList<>();
        this.rnd = rnd;
    }

    protected Gene(Gene parent){
        this(parent.variables, parent.stepwidth, parent.stepwidthChange, parent.rnd);
    }

    protected Gene createChild(){
        Gene child = new Gene(this);
        child.mutate();
        return child;
    }
        
    protected void mutate(){
        for(int i = 0; i < variables.length; i++){
            double multiplier = rnd.nextBoolean()? stepwidthChange: (1/stepwidthChange);

            stepwidth[i] *= multiplier;
            variables[i] += rnd.nextGaussian() * stepwidth[i];
        }
    }

    public void addFitness(double measurement){
        fitness.add(measurement);
    }
    
    public Double getFitness(){
        double sum=0;
        for(Double f: fitness){
            sum+=f;
        }
        return sum/fitness.size();
    }
    
    @Override
    public int compareTo(Gene o) {
        return this.getFitness().compareTo(o.getFitness());
    }       

}