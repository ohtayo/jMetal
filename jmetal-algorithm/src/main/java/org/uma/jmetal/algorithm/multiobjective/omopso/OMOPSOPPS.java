package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSOPPS extends OMOPSO {

  /** Constructor */
  public OMOPSOPPS(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                   int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                   NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
  }

  @Override
  protected void updateParticlesMemory(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      int flag = dominanceComparator.compare(swarm.get(i), localBest[i]);
      // (swarm[i] dominate localBest[i]) or (both solutions are non-dominated)
      if (flag != 1) {
        DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
        localBest[i] = particle;
      }
      // swarm[i] dominated by localBest[i]
      else{
        DoubleSolution particle = (DoubleSolution)localBest[i].copy();
        swarm.set(i, particle);
      }
    }
  }

  @Override public String getName() {
    return "OMOPSO-PPS" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO precedence pBest in the search swarm." ;
  }

}
