package org.uma.jmetal.problem.multiobjective.UF;

import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

/**
 * Class representing problem CEC2009_UF12
 */
@SuppressWarnings("serial")
public class C3_UF12MatlabEngineAtOneTimeEvaluation extends  UF12MatlabEngineAtOneTimeEvaluation {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;

 /**
  * Constructor.
  * Creates a default instance of problem CEC2009_UF12 (30 decision variables)
  */
  public C3_UF12MatlabEngineAtOneTimeEvaluation()  {
    this(30);
  }

 /**
  * Creates a new instance of problem CEC2009_UF12.
  * @param numberOfVariables Number of variables.
  */
  public C3_UF12MatlabEngineAtOneTimeEvaluation(int numberOfVariables) {
    super(numberOfVariables);
    setName("C3_UF12_Engine_AtOneTime") ;
    setNumberOfConstraints(1);

    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ;
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ;
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    super.evaluate(solution);
    this.evaluateConstraints(solution);
  }

  private void evaluateConstraints(DoubleSolution solution) {
    double[] constraint = new double[this.getNumberOfConstraints()];

    for (int j = 0; j < getNumberOfConstraints(); j++) {
/*
      constraint[j] = Math.pow(solution.getObjective(j), 2.0) / 4.0 - 1.0 ;
      double sum = 0 ;
      for (int i = 0; i < getNumberOfObjectives(); i++) {
        if (i != j) {
          sum += Math.pow(solution.getObjective(j), 2.0) ;
        }
        constraint[j]+= sum ;
      }
*/
      constraint[j] = 0.0;
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
