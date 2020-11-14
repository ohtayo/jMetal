package org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition;

import com.sun.jna.Library;
import com.sun.jna.Native;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem S_DTLZ2 with constraints.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class C3_S_DTLZ2 extends S_DTLZ2 {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;

  /**
   * Creates a default C3_S_DTLZ2 problem (30 variables, 3 objectives and 1 constraint)
   */
  public C3_S_DTLZ2() {
    this(30, 3, 1);
  }

  /**
   * Creates a C3_S_DTLZ2 problem instance
   *
   * @param numberOfVariables  Number of variables
   * @param numberOfObjectives Number of objective functions
   */
  public C3_S_DTLZ2(Integer numberOfVariables, Integer numberOfObjectives, Integer numberOfConstraints) throws JMetalException {
    super(numberOfVariables, numberOfObjectives);
    setName("C3_S_DTLZ2");

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
      double sum = 0 ;
      constraint[j] = Math.pow(solution.getObjective(j), 2.0) / 4.0 - 1.0 ;
      for (int i = 0; i < getNumberOfObjectives(); i++) {
        if (i != j) {
          sum += Math.pow(solution.getObjective(j), 2.0) ;
        }
        constraint[j]+= sum ;
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
