package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.multiobjective.ibea.IBEA;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.StrengthFitnessComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSORVDBTIBGAOP extends OMOPSORVDBTIBG {

  public DoubleSolution[] localBestArchive;

  /** Constructor */
  public OMOPSORVDBTIBGAOP(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                           int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                           NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    localBestArchive = new DoubleSolution[swarmSize];
  }

  @Override
  protected void initializeParticlesMemory(List<DoubleSolution> swarm)  {
    for (int i = 0; i < swarm.size(); i++) {
      DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
      localBest[i] = particle;
      localBestArchive[i] = (DoubleSolution) particle.copy();
    }
  }

  @Override
  protected void updateParticlesMemory(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      int flag = dominanceComparator.compare(swarm.get(i), localBest[i]);
      if (flag != 1) {
        if (flag ==-1 ){  // 優越されて更新される場合，ランクの劣るpbaestを格納
          localBestArchive[i] = (DoubleSolution) localBest[i].copy();
        }
        if (flag == 0){
          additionOldPbest(swarm, i);
        }
        DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
        localBest[i] = particle;
      }
    }
  }

  // updateされたsolutionがpBestと同一ランクの場合にこの関数を実行
  // velocityにランク落ちpBestから現在位置までのベクトルを加える
  private void additionOldPbest(List<DoubleSolution> swarm, int i) {
    DoubleSolution particle = swarm.get(i);

    // 現在位置とランク落ちpbest位置の差をとりspeedを更新
    double[] diff = new double[particle.getNumberOfVariables()];
    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      //double r = randomGenerator.nextDouble();
      double r = randomGenerator.nextDouble(0.1, 0.5);  // ランク落ちpBestからのベクトルに0.1～0.5の乱数をかける
      diff[var] =  r * ( particle.getVariableValue(var)-localBestArchive[i].getVariableValue(var) );
      speed[i][var] += diff[var];
    }

    // 差分によって現在位置も更新
    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      particle.setVariableValue(var, particle.getVariableValue(var) + diff[var]);
      if (particle.getVariableValue(var) < problem.getLowerBound(var)) {
        particle.setVariableValue(var, problem.getLowerBound(var));
        speed[i][var] = speed[i][var] * -1.0;
      }
      if (particle.getVariableValue(var) > problem.getUpperBound(var)) {
        particle.setVariableValue(var, problem.getUpperBound(var));
        speed[i][var] = speed[i][var] * -1.0;
      }
    }
    // Todo: 現在位置を更新したら本来再評価が必要．ただし，あまり評価回数が増えるのは好ましくないので，評価なしでうまく行かないか試す．
  }

  @Override public String getName() {
    return "OMOPSODBTIBGAOP" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament, Indicator Based Gbest and addition old pbest" ;
  }

}
