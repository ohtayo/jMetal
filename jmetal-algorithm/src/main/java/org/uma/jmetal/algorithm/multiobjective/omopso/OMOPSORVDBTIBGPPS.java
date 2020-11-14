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
public class OMOPSORVDBTIBGPPS extends OMOPSORVDBTIBG {
  private boolean[] updateFlags;
  private double[][] previousSpeed;

  /** Constructor */
  public OMOPSORVDBTIBGPPS(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                           int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                           NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    updateFlags = new boolean[swarmSize];
    previousSpeed = new double[swarmSize][problem.getNumberOfVariables()];
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    for (boolean flag : updateFlags) flag = true; // initialize

    double W, C1, C2;

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
      for (int a = 0; a < truncatedArchive.size(); a++) {
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
        for (int a = 0; a < truncatedArchive.size(); a++) {
          DominanceComparator<DoubleSolution> comparator = new DominanceComparator<DoubleSolution>();
          int dominated = comparator.compare(particle, truncatedArchive.get(a));
          // if particle is dominated by archive
          if (dominated == 1) {
            dominanceArchive.add(truncatedArchive.get(a));
            // if particle is same rank of archive
          } else if (dominated == 0) {
            sameRankArchive.add(truncatedArchive.get(a));
          }
        }
        // (1)[DBT] もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
        if (dominanceArchive.size() > 0) {
          updateVelocityUsingDominanceArchive(W, C1, C2, i, particle, dominanceArchive.getSolutionList(), bestParticle);
        }
        // (2)[IBG] 対象particleとarchiveが同一ランクであれば，同一ランクarchiveから近い10個体のうちランダムでvelocityを足し合わせる
        else if (sameRankArchive.size() > 0) {
          updateVelocityUsingIndicatorBasedGBest(W, C1, C2, i, particle, sameRankArchive);
        }
        // (3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
        else {
          updateVelocityUsingNoGlobalBest(W, C1, C2, i, particle, bestParticle);
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

  @Override public String getName() { return "OMOPSODBTIBGPPS"; }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament, Indicator Based Gbest and Precedence pBest in the search swarm" ;
  }

}
