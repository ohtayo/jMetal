package org.uma.jmetal.problem.multiobjective.UF;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.Cast;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem CEC2009_UF11
 */
@SuppressWarnings("serial")
public class C3_UF11 extends UF11 {

 /**
  * Constructor.
  * Creates a default instance of problem CEC2009_UF11 (30 decision variables)
  */
  public C3_UF11()  {
    this(30);
  }
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;

 /**
  * Creates a new instance of problem CEC2009_UF11.
  * @param numberOfVariables Number of variables.
  */
  public C3_UF11(int numberOfVariables) {
    super(numberOfVariables);
    setNumberOfConstraints(1) ;
    setName("C3_UF11") ;

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
