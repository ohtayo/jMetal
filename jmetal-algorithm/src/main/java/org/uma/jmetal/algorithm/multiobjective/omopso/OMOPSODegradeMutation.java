package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.List;

/** Class implementing the OMOPSO algorithm */
@SuppressWarnings("serial")
public class OMOPSODegradeMutation extends OMOPSO {

  /** Constructor */
  public OMOPSODegradeMutation(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                               int swarmSize, int maxIterations, int archiveSize, UniformMutation uniformMutation,
                               NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, archiveSize, uniformMutation, nonUniformMutation, eta);
  }

  /** Do not apply a mutation operator */
  @Override
  protected void perturbation(List<DoubleSolution> swarm)  {

  }

  @Override public String getName() {
    return "OMOPSODegradeMutation" ;
  }

  @Override public String getDescription() {
    return "OMOPSO algorithm that degraded the function of the mutation." ;
  }

}
