package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.CrowdingDistanceArchive;
import org.uma.jmetal.util.archive.impl.EpsilonCrowdingDistanceArchive;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.List;

/** Class implementing the OMOPSO algorithm */
@SuppressWarnings("serial")
public class OMOPSODegradeArchiveSize extends OMOPSO {

  /** Constructor */
  public OMOPSODegradeArchiveSize(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                                  int swarmSize, int maxIterations, int archiveSize, UniformMutation uniformMutation,
                                  NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, archiveSize, uniformMutation, nonUniformMutation, eta);
    leaderArchive = new CrowdingDistanceArchive<>(this.swarmSize);
    epsilonArchive = new EpsilonCrowdingDistanceArchive<DoubleSolution>(this.swarmSize, eta);
  }

  @Override public String getName() {
    return "OMOPSODegradeArchiveSize" ;
  }

  @Override public String getDescription() {
    return "OMOPSO algorithm that limited the size of epsilon archive." ;
  }

}
