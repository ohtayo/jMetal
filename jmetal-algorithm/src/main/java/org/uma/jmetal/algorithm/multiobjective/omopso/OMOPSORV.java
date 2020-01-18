package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSORV extends OMOPSO {

  /** Constructor */
  public OMOPSORV(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                  int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                  NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;
    DoubleSolution bestGlobal;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];

      //Select a global localBest for calculate the speed of particle i, bestGlobal
      DoubleSolution one ;
      DoubleSolution two;
      int pos1 = randomGenerator.nextInt(0, leaderArchive.getSolutionList().size() - 1);
      int pos2 = randomGenerator.nextInt(0, leaderArchive.getSolutionList().size() - 1);
      one = leaderArchive.getSolutionList().get(pos1);
      two = leaderArchive.getSolutionList().get(pos2);

      if (crowdingDistanceComparator.compare(one, two) < 1) {
        bestGlobal = one ;
      } else {
        bestGlobal = two ;
      }

      //Parameters for velocity equation
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        r1 = randomGenerator.nextDouble();
        r2 = randomGenerator.nextDouble();
        //Computing the velocity of this particle
        speed[i][var] = W * speed[i][var] + C1 * r1 * (bestParticle.getVariableValue(var) -
                particle.getVariableValue(var)) +
                C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
      }
    }
  }

  @Override public String getName() {
    return "OMOPSO-RV" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO using random vector." ;
  }

}
