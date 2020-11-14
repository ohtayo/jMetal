package org.uma.jmetal.problem.multiobjective.newDtlz;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.problem.singleobjective.Rosenbrock;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem DTLZ2 which g(x) is rosenbrock banana function.
 */
@SuppressWarnings("serial")
public class RosenbrockDTLZ2 extends AbstractDoubleProblem {
  /**
   * Creates a default DTLZ2 problem (12 variables and 3 objectives)
   */
  public RosenbrockDTLZ2() {
    this(12, 3);
  }

  /**
   * Creates a DTLZ2 problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public RosenbrockDTLZ2(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException {
    setNumberOfVariables(numberOfVariables);
    setNumberOfObjectives(numberOfObjectives);
    setName("RosenbrockDTLZ2");

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
    double[] x = new double[numberOfVariables] ;

    for (int i = 0; i < numberOfVariables; i++) {
      x[i] = solution.getVariableValue(i) ;
    }


    /*
    int k = getNumberOfVariables() - getNumberOfObjectives() + 1;
    double g = 0.0;
    for (int i = numberOfVariables - k; i < numberOfVariables; i++) {
      g += (x[i] - 0.5) * (x[i] - 0.5);
    }
    */
    // calculate g(x) as Rosenbrock function
    Rosenbrock rosenbrock = new Rosenbrock(getNumberOfVariables());
    DoubleSolution temporarySolution = new DefaultDoubleSolution((DefaultDoubleSolution) solution);
    for(int v=0; v<getNumberOfVariables(); v++) {
      double temporaryVariable = temporarySolution.getVariableValue(v);
      temporaryVariable = temporaryVariable * (rosenbrock.getUpperBound(v) - rosenbrock.getLowerBound(v)) + rosenbrock.getLowerBound(v);
      temporarySolution.setVariableValue(v, temporaryVariable);
    }
    rosenbrock.evaluate(temporarySolution);
    double g =  temporarySolution.getObjective(0);

    for (int i = 0; i < numberOfObjectives; i++) {
      f[i] = 1.0 + g;
    }

    for (int i = 0; i < numberOfObjectives; i++) {
      for (int j = 0; j < numberOfObjectives - (i + 1); j++) {
        f[i] *= Math.cos(x[j] * 0.5 * Math.PI);
      }
      if (i != 0) {
        int aux = numberOfObjectives - (i + 1);
        f[i] *= Math.sin(x[aux] * 0.5 * Math.PI);
      }
    }

    for (int i = 0; i < numberOfObjectives; i++) {
      solution.setObjective(i, f[i]);
    }
  }
}
