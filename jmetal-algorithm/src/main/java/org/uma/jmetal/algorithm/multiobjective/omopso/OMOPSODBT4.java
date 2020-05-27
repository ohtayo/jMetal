package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.StrengthFitnessComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.List;

/**
 * OMOPSOにDBTを追加したもの．
 * アーカイブはサイズ制限なしのεアーカイブと混雑距離によるリーダーアーカイブ．
 * DBT：飛翔対象解よりリーダーアーカイブに優越したものがあれば，そのなかからバイナリトーナメントでgBestを選択する手法
 */
@SuppressWarnings("serial")
public class OMOPSODBT4 extends OMOPSODBT3 {
  /** Constructor */
  public OMOPSODBT4(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                    int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                    NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
  }


  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];
      //Parameters for velocity equation
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      // 飛翔させる対象粒子とアーカイブの優越関係を確認
      NonDominatedSolutionListArchive<DoubleSolution> dominanceArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
      for (int a = 0; a < leaderArchive.size(); a++) {
        // あえてここで制約を考慮しないで優越かどうか判断させる．
        DominanceComparator<DoubleSolution> comparator = new DominanceComparator<DoubleSolution>();
        int dominated = comparator.dominanceTest(particle, leaderArchive.get(a));
        // if particle is dominated by archive
        if (dominated == 1) {
          dominanceArchive.add( leaderArchive.get(a));
        }
      }
      // (1)[DBT] もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
      if (dominanceArchive.size() > 0) {
        OMOPSODBT.updateVelocityUsingGlobalBest(W, C1, C2, r1, r2, i, particle, dominanceArchive.getSolutionList(), bestParticle, speed, crowdingDistanceComparator, randomGenerator);
      }
      // (2)[無印] 通常のOMOPSOの手法を使う
      else {
        OMOPSODBT.updateVelocityUsingGlobalBest(W, C1, C2, r1, r2, i, particle, leaderArchive.getSolutionList(), bestParticle, speed, crowdingDistanceComparator, randomGenerator);
      }
    }
  }

  @Override public String getName() {
    return "OMOPSODBT4";
  }

  @Override public String getDescription() {
    return "Optimized MOPSO using DBT.";
  }

}
