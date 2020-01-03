package org.uma.jmetal.problem.multiobjective.coco;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.problem.singleobjective.Rastrigin;
import org.uma.jmetal.problem.singleobjective.Rosenbrock;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.util.JMetalException;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem f31: Rosenbrock original / Rastrigin in the bbob-biobj test suite of the COCO platform.
 * Dimo Brockhoff , Tea Tusar, Anne Auger, Nikolaus Hansen, "Using Well-Understood Single-Objective Functions in
 * Multiobjective Black-Box Optimization Test Suites", arXiv preprint arXiv:1604.00359 (2016).
 * https://numbbo.github.io/coco-doc/bbob-biobj/functions/#rosenbrock-original-rastrigin
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class RosenbrockRastrigin extends AbstractDoubleProblem {
  /**
   * Creates a default problem (5 variables and 2 objectives)
   */
  public RosenbrockRastrigin() {
    this(5, 2);
  }

  /**
   * Creates a problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public RosenbrockRastrigin(Integer numberOfVariables, Integer numberOfObjectives) throws JMetalException {
    setNumberOfVariables(numberOfVariables);
    setNumberOfObjectives(numberOfObjectives);
    setName("RosenbrockRastrigin");

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

    for (int i = 0; i < numberOfVariables; i++) {
      x[i] = solution.getVariableValue(i) ;
    }

    // Calculate Rosenbrock function
    Rosenbrock rosenbrock = new Rosenbrock(getNumberOfVariables());
    DoubleSolution temporarySolution = new DefaultDoubleSolution((DefaultDoubleSolution) solution);
    for(int v=0; v<getNumberOfVariables(); v++) {
      double temporaryVariable = temporarySolution.getVariableValue(v);
      temporaryVariable = temporaryVariable * (rosenbrock.getUpperBound(v) - rosenbrock.getLowerBound(v)) + rosenbrock.getLowerBound(v);
      temporarySolution.setVariableValue(v, temporaryVariable);
    }
    rosenbrock.evaluate(temporarySolution);
    f[0] =  temporarySolution.getObjective(0);

    // Calculate Rastrigin function
    Rastrigin rastrigin = new Rastrigin(getNumberOfVariables());
    for(int v=0; v<getNumberOfVariables(); v++) {
      double temporaryVariable = temporarySolution.getVariableValue(v);
      temporaryVariable = temporaryVariable * (rastrigin.getUpperBound(v) - rastrigin.getLowerBound(v)) + rastrigin.getLowerBound(v);
      temporarySolution.setVariableValue(v, temporaryVariable);
    }
    rastrigin.evaluate(temporarySolution);
    f[1] =  temporarySolution.getObjective(0);

    for (int i = 0; i < numberOfObjectives; i++) {
      solution.setObjective(i, f[i]);
    }
  }
}
