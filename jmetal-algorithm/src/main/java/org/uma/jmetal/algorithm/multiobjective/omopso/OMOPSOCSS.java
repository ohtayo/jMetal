package org.uma.jmetal.algorithm.multiobjective.omopso;

import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.solution.impl.ParticleSwarmSolution;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.StrengthFitnessComparator;
import org.uma.jmetal.util.distance.Distance;
import org.uma.jmetal.util.distance.impl.EuclideanDistanceBetweenSolutionsInObjectiveSpace;
import org.uma.jmetal.util.distance.impl.EuclideanDistanceBetweenSolutionsInSolutionSpace;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.neighborhood.impl.KNearestNeighborhood;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSOCSS extends OMOPSO {
  private StrengthFitnessComparator<DoubleSolution> strengthFitnessComparator;

  /** Constructor */
  public OMOPSOCSS(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                   int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                   NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);

    strengthFitnessComparator = new StrengthFitnessComparator<DoubleSolution>();
  }


  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;

    for (int i = 0; i < swarmSize; i++) {
      ParticleSwarmSolution particle = new ParticleSwarmSolution((DefaultDoubleSolution) swarm.get(i).copy());
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];
      //Parameters for velocity equation
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      // (a) 探索初期は同一ランクの中からバイナリトーナメント選択でglobalBestを選択し，その方向を足す．
      double probability = (double)currentIteration/(double)maxIterations;
      if( probability < randomGenerator.nextDouble() ) {
        updateVelocityUsingBinaryTournamentSelection(W, C1, C2, r1, r2, i, particle, bestParticle);
      }
      // (b)探索後期はarchiveとの優越関係によって変える
      else {
        // 飛翔させる対象粒子とアーカイブの優越関係を確認
        List<DoubleSolution> dominanceArchive = new ArrayList<>();
        List<DoubleSolution> sameRankArchive = new ArrayList<>();
        for (int a = 0; a < leaderArchive.size(); a++) {
          DominanceComparator<DoubleSolution> comparator = new DominanceComparator<>();
          int dominated = comparator.compare(particle, leaderArchive.get(a));
          // if particle is dominated by archive
          if (dominated == 1) {
            dominanceArchive.add((DoubleSolution) leaderArchive.get(a).copy());
            // if particle is same rank of archive
          } else if (dominated == 0) {
            sameRankArchive.add((DoubleSolution) leaderArchive.get(a).copy());
          }
        }
        // (b-1) もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
        if (dominanceArchive.size() > 0) {
          updateVelocityUsingDominanceArchive(W, C1, C2, r1, r2, i, particle, dominanceArchive, bestParticle);
        }
        // (b-2) 対象particleとarchiveが同一ランクであれば，同一ランクarchiveから密度の低い箇所をgBestとして探索する
        else if (sameRankArchive.size() > 0) {
          updateVelocityUsingSameRankArchiveCoarseSpace(W, C1, C2, r1, r2, i, particle, sameRankArchive);  // (b-2-II)の手法を用いる
        }
        // (b-3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
        else {
          updateVelocityUsingNoGlobalBest(W, C1, C2, r1, r2, i, particle, bestParticle);
        }
      }
    }
  }

  // (a) : 通常のPSOと同様
  protected void updateVelocityUsingBinaryTournamentSelection(
          double W, double C1, double C2, double r1, double r2, int i, DoubleSolution particle, DoubleSolution bestParticle
  ){
    // random select
    DoubleSolution one ;
    DoubleSolution two;
    int pos1 = randomGenerator.nextInt(0, leaderArchive.getSolutionList().size() - 1);
    int pos2 = randomGenerator.nextInt(0, leaderArchive.getSolutionList().size() - 1);
    one = leaderArchive.getSolutionList().get(pos1);
    two = leaderArchive.getSolutionList().get(pos2);

    // binary tournament selection using strength raw fitness
    DoubleSolution bestGlobal;
    if (strengthFitnessComparator.compare(one, two) < 1) {
      bestGlobal = one ;
    } else {
      bestGlobal = two ;
    }

    // update speed
    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      //Computing the velocity of this particle
      speed[i][var] =
              W * speed[i][var]
                      + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
                      + C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
    }

  }

  // (b-1) もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
  protected void updateVelocityUsingDominanceArchive(
          double W, double C1, double C2, double r1, double r2, int i, DoubleSolution particle, List<DoubleSolution> dominanceArchive, DoubleSolution bestParticle
  ){
    if ( dominanceArchive.size()==1 ){
      dominanceArchive.add(dominanceArchive.get(0));
    }

    // random select
    DoubleSolution one ;
    DoubleSolution two;
    int pos1 = randomGenerator.nextInt(0, dominanceArchive.size() - 1);
    int pos2 = randomGenerator.nextInt(0, dominanceArchive.size() - 1);
    one = dominanceArchive.get(pos1);
    two = dominanceArchive.get(pos2);

    // binary tournament selection using strength raw fitness
    DoubleSolution bestGlobal;
    if (strengthFitnessComparator.compare(one, two) < 1) {
      bestGlobal = one ;
    } else {
      bestGlobal = two ;
    }

    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      //Computing the velocity of this particle
      speed[i][var] =
              W * speed[i][var]
                      + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
                      + C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
    }

  }
  // listにsolutionを含むかチェック
  private boolean contains(List<DoubleSolution> list, DoubleSolution solution, int[] index){
    for( int s=0; s<list.size(); s++) {
      if (list.get(s).equals(solution)) {
        index[0] = s;
        return true;
      }
    }
    return false;
  }
  // (b-2-II) 対象particleとarchiveが同一ランクであれば
  // 同一ランクarchiveから近い10個体のVelocityで飛翔した位置の平均をとり，それをgbestとする
  protected void updateVelocityUsingSameRankArchiveCoarseSpace(
          double W, double C1, double C2, double r1, double r2, int i, DoubleSolution particle, List<DoubleSolution> sameRankArchive
  ) {
    DoubleSolution bestParticle = localBest[i];
    List<DoubleSolution> neighbors;
    DoubleSolution bestGlobal;

    if(sameRankArchive.size()==1){
      bestGlobal = sameRankArchive.get(0);
    }else{
      // SameRankArchive同士のEuclid distanceを計算する
      Distance<DoubleSolution, DoubleSolution> distance = new EuclideanDistanceBetweenSolutionsInObjectiveSpace<>();
      Matrix distances = new Matrix( sameRankArchive.size(),sameRankArchive.size() );
      for(int s1=0; s1<sameRankArchive.size(); s1++){
        for(int s2=0; s2<sameRankArchive.size(); s2++){
          if(s1==s2) {
            distances.set( s1, s2, Double.POSITIVE_INFINITY );
          }else{
            distances.set( s1, s2, distance.getDistance(sameRankArchive.get(s1), sameRankArchive.get(s2)) );
          }
        }
      }
      // ほかの解のEuclid distanceのうち最小値が最も大きいものを抽出してglobal bestにする
      Vector minimumDistances = distances.min(Matrix.DIRECTION_ROW);
      int[] gBestIndex = new int[1];
      minimumDistances.max(gBestIndex);
      bestGlobal = sameRankArchive.get(gBestIndex[0]);
    }

    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      //Computing the velocity of this particle
      speed[i][var] =
              W * speed[i][var]
                      + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
                      + C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
    }
  }

  // (b-3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
  protected void updateVelocityUsingNoGlobalBest(
          double W, double C1, double C2, double r1, double r2, int i, ParticleSwarmSolution particle, DoubleSolution bestParticle )
  {
    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      //Computing the velocity of this particle
      speed[i][var] =
              W * speed[i][var]
                      + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
                      + C2 * r2 * speed[i][var];
    }
  }


  @Override public String getName() {
    return "OMOPSO-PPS" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO precedence pBest in the search swarm." ;
  }

}