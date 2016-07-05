package agentAJJT.agents;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import agentIO.PerceptorInput;
import agentIO.ServerCommunication;

/**
 * Vereint die Rohdaten des Accelerometers und des Gyrometers.
 */
public class OrientationPerceptor {
    
    private PerceptorInput in;
    private Vector3D result_prev;

    public OrientationPerceptor(PerceptorInput in) {
        this.in = in;
        this.result_prev = new Vector3D(0.0, 0.0, 0.0);       
    }

    /**
     * Gibt die mit dem Komplementärfilter kombinierte Orientierung zurück.
     */
    public Vector3D getOrientation(double lambda) {
	// assert 0 <= lambda <= 1
        Vector3D a = in.getAcc();
        Vector3D delta_g = in.getGyro().scalarMultiply(0.02);
        Vector3D result;
        
        // Komplementärfilter                
        // Merke: a.getX() und a.getY() sind vertauscht!
        result = new Vector3D(
               (1 - lambda) * a.getY() + lambda * (delta_g.getX() + result_prev.getX()),
               (1-lambda)*a.getX() + lambda * (delta_g.getY() + result_prev.getY()),
               (1-lambda)*a.getZ() + lambda * (delta_g.getZ() + result_prev.getZ())        
        );
        
        // Wert merken
        result_prev = result;
        
	return result;
    }
}