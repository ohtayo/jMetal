package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSOAOP extends OMOPSO {

  List<List<DoubleSolution>> oldBestParticleListSwarm;
  /** Constructor */
  public OMOPSOAOP(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                   int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                   NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    oldBestParticleListSwarm = new ArrayList<>();
  }

  @Override
  protected void initializeParticlesMemory(List<DoubleSolution> swarm)  {
    for (int i = 0; i < swarm.size(); i++) {
      DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
      localBest[i] = particle;
      List<DoubleSolution> temp = new ArrayList<>();
      temp.add(particle);
      oldBestParticleListSwarm.add(temp);
    }
  }

  @Override
  protected void updateParticlesMemory(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      int flag = dominanceComparator.compare(swarm.get(i), localBest[i]);
      if (flag != 1) {
        DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
        localBest[i] = particle;
        oldBestParticleListSwarm.get(i).add(particle); // pBestが更新されたらpBestを過去リストに格納
      }
    }
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, r3, W, C1, C2, C3;
    DoubleSolution bestGlobal;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];

      // Select global best for calculate the speed of particle
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

      // Select personal best
      List<DoubleSolution> oldBestParticleList = oldBestParticleListSwarm.get(i);
      int pos3;
      if(oldBestParticleList.size() > 1) {
        pos3 = randomGenerator.nextInt(0, oldBestParticleList.size() - 2);
      }else{
        pos3 = randomGenerator.nextInt(0, oldBestParticleList.size() - 1);
      }
      DoubleSolution oldBestParticle = oldBestParticleList.get(pos3);

      // Parameters for velocity equation
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      r3 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      C3 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      // Compute the velocity of this particle
      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        speed[i][var] =
                W * speed[i][var]
              + C1 * r1 * ( bestParticle.getVariableValue(var)-particle.getVariableValue(var)
                            + ( C3*r3*(oldBestParticle.getVariableValue(var)-particle.getVariableValue(var)) )
                          )
              + C2 * r2 * ( bestGlobal.getVariableValue(var)-particle.getVariableValue(var) );
      }
    }
  }


  @Override public String getName() {
    return "OMOPSO-AOP" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO added old pBest" ;
  }

}
