package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSORVPPS extends OMOPSORV {
  private boolean[] updateFlags;
  private double[][] previousSpeed;
  public double eta;
  /** Constructor */
  public OMOPSORVPPS(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                     int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                     NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    updateFlags = new boolean[swarmSize];
    previousSpeed = new double[swarmSize][problem.getNumberOfVariables()];
    this.eta = eta;
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    for (boolean flag : updateFlags) flag = true; // initialize
    DoubleSolution bestGlobal;
    double W, C1, C2, r1, r2;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];
      //Parameters for velocity equation
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      // 飛翔させる対象粒子とアーカイブの優越関係を確認
      NonDominatedSolutionListArchive<DoubleSolution> dominanceArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
      List<DoubleSolution> sameRankArchive = new ArrayList<>(archiveSize);

      // pBestがアーカイブに優越しているかチェック．優越していたらvelocityは更新しない．
      for (int a = 0; a < epsilonArchive.size(); a++) {
        DominanceComparator<DoubleSolution> comparator = new DominanceComparator<DoubleSolution>();
        int dominated = comparator.compare(particle, bestParticle);
        // アーカイブにpBestを優越する解が1つもなく，かつアーカイブの解を1つでもpBestが優越していたら，pBest位置と速度を固定する
        if (dominated == -1) {  // アーカイブの1つの解がpbestを優越していたら
          updateFlags[i] = true;
          break;
        }else if(dominated == 1){   // アーカイブの解がpBestに優越されていたら
          updateFlags[i] = false;
        }
      }
      if(updateFlags[i]) {
        // 更新フラグがあれば，通常のOMOPSOの更新をする．
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

  }

  /** Update the position of each particle */
  @Override
  protected void updatePosition(List<DoubleSolution> swarm)  {
    for (int i = 0; i < swarmSize; i++) {
      // pBestがアーカイブに優越していたら位置は更新しない．
      if(updateFlags[i]) {
        DoubleSolution particle = swarm.get(i);
        for (int var = 0; var < particle.getNumberOfVariables(); var++) {
          particle.setVariableValue(var, particle.getVariableValue(var) + speed[i][var]);
          if (particle.getVariableValue(var) < problem.getLowerBound(var)) {
            particle.setVariableValue(var, problem.getLowerBound(var));
            speed[i][var] = speed[i][var] * -1.0;
          }
          if (particle.getVariableValue(var) > problem.getUpperBound(var)) {
            particle.setVariableValue(var, problem.getUpperBound(var));
            speed[i][var] = speed[i][var] * -1.0;
          }
        }
      }
      else{
        DoubleSolution particle = swarm.get(i);
        for (int var = 0; var < particle.getNumberOfVariables(); var++) {
          particle.setVariableValue(var, ((DoubleSolution)localBest[i]).getVariableValue(var));
        }
      }
    }
  }

  @Override public String getName() { return "OMOPSORVPPS"; }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament, Indicator Based Gbest and Precedence pBest in the search swarm" ;
  }

}
