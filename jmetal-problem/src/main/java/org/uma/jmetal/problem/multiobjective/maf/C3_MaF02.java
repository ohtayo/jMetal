package org.uma.jmetal.problem.multiobjective.maf;

import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem C3_MaF02, C3_DTLZ2BZ
 */
@SuppressWarnings("serial")
public class C3_MaF02 extends MaF02 {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;

  /**
   * Default constructor
   */
  public C3_MaF02() {
    this(12, 3, 1) ;
  }

  /**
   * Creates a MaF02 problem instance
   *
   * @param numberOfVariables Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public C3_MaF02(Integer numberOfVariables,
                  Integer numberOfObjectives,
                  Integer numberOfConstraints) {
    super(numberOfVariables, numberOfObjectives);
    setName("C3_MaF02");
    setNumberOfConstraints(numberOfConstraints);

    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ;
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ;
  }

  /**
   * Evaluates a solution
   *
   * @param solution The solution to evaluate
   */
  @Override
  public void evaluate(DoubleSolution solution) {
    super.evaluate(solution);
    this.evaluateConstraints(solution);
  }

  private void evaluateConstraints(DoubleSolution solution) {
    double[] constraint = new double[this.getNumberOfConstraints()];

    for (int j = 0; j < getNumberOfConstraints(); j++) {
      constraint[j] = 0.0;
      /*
      for (int i = 0; i < getNumberOfObjectives(); i++) {
        if (i != j) {
          constraint[j] += solution.getObjective(j) + solution.getObjective(i) / 0.5 - 1.0;
        }
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
