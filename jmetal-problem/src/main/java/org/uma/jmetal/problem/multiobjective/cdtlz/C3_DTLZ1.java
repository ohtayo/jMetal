package org.uma.jmetal.problem.multiobjective.cdtlz;

import org.uma.jmetal.problem.multiobjective.dtlz.DTLZ1;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

/**
 * Problem C3-DTLZ1, defined in:
 * Jain, H. and K. Deb.  "An Evolutionary Many-Objective Optimization Algorithm Using Reference-Point-Based
 * Nondominated Sorting Approach, Part II: Handling Constraints and Extending to an Adaptive Approach."
 * EEE Transactions on Evolutionary Computation, 18(4):602-622, 2014.
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
@SuppressWarnings("serial")
public class C3_DTLZ1 extends DTLZ1 {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  /**
   * Creates a default C3_DTLZ1 problem (n=8 variables (M+4), M=4 objectives and 1 constraints)
   */
  public C3_DTLZ1() {
    this(8, 4, 1);
  }

  /**
   * Constructor
   * @param numberOfVariables
   * @param numberOfObjectives
   */
  public C3_DTLZ1(Integer numberOfVariables, Integer numberOfObjectives, Integer numberOfConstraints) {
    super(numberOfVariables, numberOfObjectives) ;
    setName("C3_DTLZ1");

    setNumberOfConstraints(numberOfConstraints);

    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ;
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ;
  }

  @Override
  public void evaluate(DoubleSolution solution) {
    super.evaluate(solution);
    this.evaluateConstraints(solution);
  }

  private void evaluateConstraints(DoubleSolution solution) {
    double[] constraint = new double[this.getNumberOfConstraints()];

    for (int j = 0; j < getNumberOfConstraints(); j++) {
      constraint[j] = 0.0;
      for (int i = 0; i < getNumberOfObjectives(); i++) {
        if (i != j) {
          constraint[j] += solution.getObjective(j) + solution.getObjective(i) / 0.5 - 1.0;
        }

      }
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
