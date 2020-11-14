package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.List;

/** Class implementing the OMOPSO algorithm */
@SuppressWarnings("serial")
public class OMOPSODegradeSelection extends OMOPSO {

  /** Constructor */
  public OMOPSODegradeSelection(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
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

      // Select a global best by random selection, not Binary tournament selection.
      int pos = randomGenerator.nextInt(0, leaderArchive.getSolutionList().size() - 1);
      bestGlobal = leaderArchive.getSolutionList().get(pos);

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
    return "OMOPSODegradeSelection" ;
  }

  @Override public String getDescription() {
    return "OMOPSO algorithm that degraded the function of gBest selection." ;
  }

}
