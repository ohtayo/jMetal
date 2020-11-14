package org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition;

import com.sun.jna.Library;
import com.sun.jna.Native;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Problem R_DTLZ2, defined in:
 * V. L. Huang and A. K. Qin and K. Deb and E. Zitzler and P. N. Suganthan and J. J Liang and M. Preuss and S. Huband,
 * "Problem definitions for performance assessment of multi-objective optimization algorithms", IEEE Congress on Evolutionary Computation 2007(CEC2007), 2007
 * Build 64 bit DLL from C source code of <https://github.com/P-N-Suganthan/CEC2007>.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class R_DTLZ2 extends AbstractDoubleProblem {

    /**
     * JNA Interface (needs fsuite64.dll in path)
   */
  public interface CEC2007TestSuite extends Library {
    CEC2007TestSuite INSTANCE = (CEC2007TestSuite) Native.load("fsuite64", CEC2007TestSuite.class);
    void R_DTLZ2(double[] x, double[] f, int nx, int n_obj);
  }

  /**
   * Creates a default R_DTLZ2 problem (30 variables and 3 objectives)
   */
  public R_DTLZ2() {
    this(30, 3);
  }

  /**
   * Creates a R_DTLZ2 problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public R_DTLZ2(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException {
    setNumberOfVariables(numberOfVariables);
    setNumberOfObjectives(numberOfObjectives);
    setName("R_DTLZ2");

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

    double[] x_min = {-1.773,-1.846,-1.053,-2.37,-1.603,-1.878,-1.677,-0.935,-1.891,-0.964,-0.885,-1.69,-2.235,-1.541,-0.72,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double[] x_max = {1.403,1.562,2.009,0.976,1.49,1.334,1.074,2.354,1.462,2.372,2.267,1.309,0.842,1.665,2.476,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};

    for (int v = 0; v < numberOfVariables; v++) {
      x[v] = solution.getVariableValue(v) * (x_max[v]-x_min[v]) + x_min[v] ;
    }

//  Call function using JNA
    CEC2007TestSuite testSuite = CEC2007TestSuite.INSTANCE;
    testSuite.R_DTLZ2(x, f, numberOfVariables, numberOfObjectives);

    for (int n = 0; n < numberOfObjectives; n++) {
      solution.setObjective(n, f[n]);
    }
  }
}
