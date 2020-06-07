package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.impl.AbstractParticleSwarmOptimization;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.archive.Archive;
import org.uma.jmetal.util.archive.impl.CrowdingDistanceArchive;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.CrowdingDistanceComparator;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.solutionattribute.impl.CrowdingDistance;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Class implementing the OMOPSO algorithm */
@SuppressWarnings("serial")
public class OMOPSODegradeLeader extends OMOPSO {

  /** Constructor */
  public OMOPSODegradeLeader(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                             int swarmSize, int maxIterations, int archiveSize, UniformMutation uniformMutation,
                             NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, archiveSize, uniformMutation, nonUniformMutation, eta);
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;
    DoubleSolution bestGlobal;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];

      DoubleSolution one ;
      DoubleSolution two;

      // select a global best from the epsilonArchive, not the leaderArchive
      int pos1 = randomGenerator.nextInt(0, epsilonArchive.getSolutionList().size() - 1);
      int pos2 = randomGenerator.nextInt(0, epsilonArchive.getSolutionList().size() - 1);
      one = epsilonArchive.getSolutionList().get(pos1);
      two = epsilonArchive.getSolutionList().get(pos2);

      if (crowdingDistanceComparator.compare(one, two) < 1) {
        bestGlobal = one ;
      } else {
        bestGlobal = two ;
      }

      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        speed[i][var] = W * speed[i][var] + C1 * r1 * (bestParticle.getVariableValue(var) -
            particle.getVariableValue(var)) +
            C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
      }
    }
  }

  @Override public String getName() {
    return "OMOPSODegradeLeader" ;
  }

  @Override public String getDescription() {
    return "OMOPSO algorithm that degraded the function of the leader archive." ;
  }

}
