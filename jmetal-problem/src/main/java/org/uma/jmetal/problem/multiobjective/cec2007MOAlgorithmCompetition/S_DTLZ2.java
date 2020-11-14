package org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition;

import com.sun.jna.Native;
import com.sun.jna.Library;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Problem S_DTLZ2, defined in:
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
   * Creates a default S_DTLZ2 problem (30 variables and 3 objectives)
   */
  public S_DTLZ2() {
    this(30, 3);
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

    double[] x_min = {0.211,0.184,0.667,0.695,0.88,0.355,-0.152,0.445,0.597,0.894,0.061,0.647,0.353,0.861,0.578,0.114,0.209,0.043,0.739,0.175,-0.146,0.697,0.743,0.179,0.132,0.714,-0.072,0.729,0.576,0.61};
    double[] x_max = {1.366,1.303,1.852,1.759,1.95,1.558,1.014,1.596,1.816,1.977,1.222,1.704,1.522,1.933,1.713,1.228,1.45,1.172,1.969,1.356,1.049,1.755,1.895,1.286,1.251,1.933,1.131,1.941,1.702,1.848};

    for (int v = 0; v < numberOfVariables; v++) {
      x[v] = solution.getVariableValue(v) * (x_max[v]-x_min[v]) + x_min[v] ;
    }

//  Call S_DTLZ2 function using JNA
    CEC2007TestSuite testSuite = CEC2007TestSuite.INSTANCE;
    testSuite.S_DTLZ2(x, f, numberOfVariables, numberOfObjectives);

/*
// Calculate S_DTLZ2 function----------------------------------
    int nx = numberOfVariables;
    int n_obj = numberOfObjectives;
    int k = nx - n_obj + 1;
    double g = 0;
    double[] z = new double[nx];
    double[] zz = new double[nx];
    double[] p = new double[nx];
    double[] psum = new double[n_obj];
    double[] o={0.366, 0.303, 0.852, 0.759, 0.950, 0.558, 0.014, 0.596, 0.816, 0.977, 0.222, 0.704, 0.522, 0.933, 0.713, 0.228, 0.450, 0.172, 0.969, 0.356, 0.049, 0.755, 0.895, 0.286, 0.251, 0.933, 0.131, 0.941, 0.702, 0.848};
    double[] d_l={0.155, 0.119, 0.185, 0.064, 0.07, 0.203, 0.166, 0.151, 0.219, 0.083, 0.161, 0.057, 0.169, 0.072, 0.135, 0.114, 0.241, 0.129, 0.23, 0.181, 0.195, 0.058, 0.152, 0.107, 0.119, 0.219, 0.203, 0.212, 0.126, 0.238};
    double[] lamda_l={3.236, 4.201, 2.701, 7.775, 7.148, 2.465, 3.02, 3.322, 2.285, 6.004, 3.106, 8.801, 2.956, 6.918, 3.708, 4.403, 2.077, 3.884, 2.171, 2.76, 2.567, 8.636, 3.299, 4.666, 4.208, 2.285, 2.469, 2.362, 3.963, 2.099};

    for(int i = 0; i < nx; i++)
    {
      z[i]=x[i]-o[i];

      if (z[i] < 0)
      {
        zz[i]=-lamda_l[i] * z[i];
        p[i]=-z[i]/d_l[i];
      }
      else
      {
        zz[i]=z[i];
        p[i] = 0;
      }
    }
    for(int j = 0; j < n_obj; j++)
    {
      psum[j] = 0;
    }
    for (int i = nx - k + 1; i <= nx; i++)
    {
      g += Math.pow(zz[i-1]-0.5,2.0);
      for(int j = 0; j < n_obj; j++)
      {
        psum[j]= Math.sqrt(Math.pow(psum[j],2.0) + Math.pow(p[i-1],2.0));
      }
    }
    for (int i = 1; i <= n_obj; i++)
    {
      double ff = (1 + g);
      for (int j = n_obj - i; j >= 1; j--)
      {
        ff *= Math.cos(zz[j-1] * Math.PI / 2.0);
        psum[i-1] = Math.sqrt( Math.pow(psum[i-1],2.0) + Math.pow(p[j-1],2.0) );
      }
      if (i > 1)
      {
        ff *= Math.sin(zz[(n_obj - i + 1) - 1] * Math.PI / 2.0);
        psum[i-1] = Math.sqrt( Math.pow(psum[i-1],2.0) + Math.pow(p[(n_obj - i + 1) - 1],2.0) );
      }

      f[i-1] =  2.0/(1+Math.exp(-psum[i-1])) * (ff+1);
    }
    // Calculate S_DTLZ2 function----------------------------------
*/

    for (int n = 0; n < numberOfObjectives; n++) {
      solution.setObjective(n, f[n]);
    }
  }
}
