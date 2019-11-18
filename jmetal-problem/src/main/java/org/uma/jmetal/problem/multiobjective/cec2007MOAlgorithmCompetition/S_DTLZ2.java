package org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition;

import com.sun.jna.Native;
import com.sun.jna.Library;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Problem S_DTLZ4, defined in:
 * V. L. Huang and A. K. Qin and K. Deb and E. Zitzler and P. N. Suganthan and J. J Liang and M. Preuss and S. Huband,
 * "Problem definitions for performance assessment of multi-objective optimization algorithms", IEEE Congress on Evolutionary Computation 2007(CEC2007), 2007
 * Build 64 bit DLL from C source code of <https://github.com/P-N-Suganthan/CEC2007>.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class S_DTLZ2 extends AbstractDoubleProblem {

  /**
   * JNA Interface (needs fsuite64.dll in path)
   */
  public interface CEC2007TestSuite extends Library {
    CEC2007TestSuite INSTANCE = (CEC2007TestSuite) Native.load("fsuite64", CEC2007TestSuite.class);
    void S_DTLZ2(double[] x, double[] f, int nx, int n_obj);
  }

  /**
   * Creates a default S_DTLZ2 problem (12 variables and 3 objectives)
   */
  public S_DTLZ2() {
    this(12, 3);
  }

  /**
   * Creates a S_DTLZ2 problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public S_DTLZ2(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException {
    setNumberOfVariables(numberOfVariables);
    setNumberOfObjectives(numberOfObjectives);
    setName("S_DTLZ2");

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

    for (int v = 0; v < numberOfVariables; v++) {
      x[v] = solution.getVariableValue(v) ;
    }

//  Call S_DTLZ2 function using JNA
    CEC2007TestSuite testSuite = CEC2007TestSuite.INSTANCE;
    testSuite.S_DTLZ2(x, f, numberOfVariables, numberOfObjectives);

    for (int v = 0; v < numberOfObjectives; v++) {
      solution.setObjective(v, f[v]);
    }
  }
}
