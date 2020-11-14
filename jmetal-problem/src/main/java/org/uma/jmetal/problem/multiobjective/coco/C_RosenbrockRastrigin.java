package org.uma.jmetal.problem.multiobjective.coco;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.problem.singleobjective.Rastrigin;
import org.uma.jmetal.problem.singleobjective.Rosenbrock;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem f31: Rosenbrock original / Rastrigin with constraint(s).
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class C_RosenbrockRastrigin extends RosenbrockRastrigin {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  /**
   * Creates a default problem (default: 12 variables, 2 objectives and 1 constraint)
   */
  public C_RosenbrockRastrigin() {
    this(12, 2, 1);
  }

  /**
   * Creates a problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public C_RosenbrockRastrigin(Integer numberOfVariables, Integer numberOfObjectives, Integer numberOfConstraints) throws JMetalException {
    super(numberOfVariables, numberOfObjectives);
    setName("C_RosenbrockRastrigin");
    setNumberOfConstraints(numberOfConstraints);

    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ;
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ;
  }

  /** Evaluate() method */
  public void evaluate(DoubleSolution solution) {
    super.evaluate(solution);
    this.evaluateConstraints(solution);
  }

  private void evaluateConstraints(DoubleSolution solution) {
    double[] constraint = new double[this.getNumberOfConstraints()];

    for (int j = 0; j < getNumberOfConstraints(); j++) {
      constraint[j] = 0.0;
      /*
      double sum = 0 ;
      constraint[j] = Math.pow(solution.getObjective(j), 2.0) / 4.0 - 1.0 ;
      for (int i = 0; i < getNumberOfObjectives(); i++) {
        if (i != j) {
          sum += Math.pow(solution.getObjective(j), 2.0) ;
        }
        constraint[j]+= sum ;
      }
      */
    }

    double overallConstraintViolation = 0.0;
    int violatedConstraints = 0;
    for (int i = 0; i < getNumberOfConstraints(); i++) {
      if (constraint[i]<0.0){
        overallConstraintViolation+=constraint[i];
        violatedConstraints++;
      }
    }

    overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolation);
    numberOfViolatedConstraints.setAttribute(solution, violatedConstraints);
  }

}
