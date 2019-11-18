package org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition;

import com.sun.jna.Library;
import com.sun.jna.Native;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Problem S_DTLZ3, defined in:
 * V. L. Huang and A. K. Qin and K. Deb and E. Zitzler and P. N. Suganthan and J. J Liang and M. Preuss and S. Huband,
 * "Problem definitions for performance assessment of multi-objective optimization algorithms", IEEE Congress on Evolutionary Computation 2007(CEC2007), 2007
 * Build 64 bit DLL from C source code of <https://github.com/P-N-Suganthan/CEC2007>.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class S_DTLZ3 extends AbstractDoubleProblem {

    /**
     * JNA Interface (needs fsuite64.dll in path)
   */
  public interface CEC2007TestSuite extends Library {
    CEC2007TestSuite INSTANCE = (CEC2007TestSuite) Native.load("fsuite64", CEC2007TestSuite.class);
    void S_DTLZ3(double[] x, double[] f, int nx, int n_obj);
  }

  /**
   * Creates a default S_DTLZ3 problem (30 variables and 3 objectives)
   */
  public S_DTLZ3() {
    this(30, 3);
  }

  /**
   * Creates a S_DTLZ3 problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public S_DTLZ3(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException {
    setNumberOfVariables(numberOfVariables);
    setNumberOfObjectives(numberOfObjectives);
    setName("S_DTLZ3");

    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;

    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(0.0);
      upperLimit.add(1.0);
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);
  }

  /** Evaluate() method */
  public void evaluate(DoubleSolution solution) {
    int numberOfVariables = getNumberOfVariables();
    int numberOfObjectives = getNumberOfObjectives();
    double[] f = new double[numberOfObjectives];
    double[] x = new double[numberOfVariables];

    double[] x_min = {0.054,0.336,-0.104,0.787,0.492,0.116,0.209,0.717,0.153,-0.009,0.039,-0.008,0.398,0.05,0.387,0.003,0.529,0.246,0.593,-0.134,0.403,0.891,0.137,0.782,-0.017,-0.154,0.031,0.721,-0.063,0.026};
    double[] x_max = {1.209,1.455,1.081,1.851,1.562,1.319,1.375,1.868,1.372,1.074,1.2,1.049,1.567,1.122,1.522,1.117,1.77,1.375,1.823,1.047,1.598,1.949,1.289,1.889,1.102,1.065,1.234,1.933,1.063,1.264};

    for (int v = 0; v < numberOfVariables; v++) {
      x[v] = solution.getVariableValue(v) * (x_max[v]-x_min[v]) + x_min[v] ;
    }

//  Call S_DTLZ3 function using JNA
    CEC2007TestSuite testSuite = CEC2007TestSuite.INSTANCE;
    testSuite.S_DTLZ3(x, f, numberOfVariables, numberOfObjectives);

    for (int n = 0; n < numberOfObjectives; n++) {
      solution.setObjective(n, f[n]);
    }
  }
}
