package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSORVDBT extends OMOPSORV {
  private double eta;
  /** Constructor */
  public OMOPSORVDBT(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                  int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                  NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    this.eta = eta;
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;
    DoubleSolution bestGlobal;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];
      //Parameters for velocity equation
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);
      // 飛翔させる対象粒子とアーカイブの優越関係を確認
      NonDominatedSolutionListArchive<DoubleSolution> dominanceArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
      for (int a = 0; a < leaderArchive.size(); a++) {
        DominanceComparator<DoubleSolution> comparator = new DominanceComparator<DoubleSolution>();
        int dominated = comparator.compare(particle, leaderArchive.get(a));
        // if particle is dominated by archive
        if (dominated == 1) {
          dominanceArchive.add( leaderArchive.get(a));
          // if particle is same rank of archive
        }
      }
      // (1)[DBT] もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
      if (dominanceArchive.size() > 0) {
        updateVelocityUsingDominanceArchive(W, C1, C2, i, particle, dominanceArchive.getSolutionList(), bestParticle);
      }
      // (2)[無印] 対象particleとarchiveが同一ランクであれば，同一ランクarchiveから近い10個体のうちランダムでvelocityを足し合わせる
      else {
        updateVelocityUsingGlobalBest(W, C1, C2, i, particle, bestParticle);
      }
    }
  }

  // (1)[DBT] もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
  protected void updateVelocityUsingDominanceArchive(
          double W, double C1, double C2, int i, DoubleSolution particle, List<DoubleSolution> dominanceArchive, DoubleSolution bestParticle
  ){
    double r1, r2;
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
    if (crowdingDistanceComparator.compare(one, two) < 1) {
      bestGlobal = one ;
    } else {
      bestGlobal = two ;
    }

    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      //Computing the velocity of this particle
      speed[i][var] =
              W * speed[i][var]
                      + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
                      + C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
    }
  }

  // [無印] 通常のOMOPSOの飛翔をする．
  protected void updateVelocityUsingGlobalBest(
          double W, double C1, double C2, int i, DoubleSolution particle, DoubleSolution bestParticle )
  {
    double r1, r2;
    DoubleSolution bestGlobal;
    //Select a global localBest for calculate the speed of particle i, bestGlobal
    DoubleSolution one ;
    DoubleSolution two;
    int pos1 = randomGenerator.nextInt(0, leaderArchive.size() - 1);
    int pos2 = randomGenerator.nextInt(0, leaderArchive.size() - 1);
    one = leaderArchive.get(pos1);
    two = leaderArchive.get(pos2);

    if (crowdingDistanceComparator.compare(one, two) < 1) {
      bestGlobal = one ;
    } else {
      bestGlobal = two ;
    }

    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      //Computing the velocity of this particle
      speed[i][var] = W * speed[i][var] + C1 * r1 * (bestParticle.getVariableValue(var) -
              particle.getVariableValue(var)) +
              C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
    }
  }

  @Override public String getName() {
    return "OMOPSORVDBT" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO using random vector." ;
  }

}
