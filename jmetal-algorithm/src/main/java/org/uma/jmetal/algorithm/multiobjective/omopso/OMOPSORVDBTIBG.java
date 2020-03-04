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
public class OMOPSORVDBTIBG extends OMOPSOWithSizeLimitedArchive {

  public NonDominatedSolutionListArchive<DoubleSolution> temporaryArchive; // 一時アーカイブ
  public List<DoubleSolution> truncatedArchive; // SPEA2の端切りアーカイブ
  public EnvironmentalSelection<DoubleSolution> environmentalSelection; // SPEA2の環境選択
  public StrengthRawFitness<DoubleSolution> strengthRawFitness; // SPEA2の適合度
  private StrengthFitnessComparator<DoubleSolution> strengthFitnessComparator;
  /** Constructor */
  public OMOPSORVDBTIBG(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                        int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                        NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);

    truncatedArchive = new ArrayList<DoubleSolution>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<DoubleSolution>();
    environmentalSelection = new EnvironmentalSelection<DoubleSolution>(archiveSize);
    strengthFitnessComparator = new StrengthFitnessComparator<DoubleSolution>();
  }

  @Override protected void initProgress() {
    currentIteration = 1;
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
  }

  @Override protected void updateProgress() {
    currentIteration += 1;
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
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
      for (int a = 0; a < truncatedArchive.size(); a++) {
        DominanceComparator<DoubleSolution> comparator = new DominanceComparator<DoubleSolution>();
        int dominated = comparator.compare(particle, truncatedArchive.get(a));
        // if particle is dominated by archive
        if (dominated == 1) {
          dominanceArchive.add( truncatedArchive.get(a));
          // if particle is same rank of archive
        } else if (dominated == 0) {
          sameRankArchive.add( truncatedArchive.get(a));
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
    if (strengthFitnessComparator.compare(one, two) < 1) {
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

  // [IBG]同一ランクの中でIBEAのFitnessを計算し，ランキング1位をgBestとして交叉する．
  protected void updateVelocityUsingIndicatorBasedGBest(
      double W, double C1, double C2, int i, DoubleSolution particle, List<DoubleSolution> sameRankArchive
  ) {
    double r1, r2;
    DoubleSolution bestParticle = localBest[i];
    DoubleSolution bestGlobal;

    // 同一ランクのアーカイブが1つなら，それがgBest
    if(sameRankArchive.size()==1){
      bestGlobal = sameRankArchive.get(0);
    }else {
      // IBEAをインスタンス化してcalculateFitnessをする
      IBEA<DoubleSolution> ibea = new IBEA<DoubleSolution>(problem, swarmSize, archiveSize, maxIterations,
              new BinaryTournamentSelection<DoubleSolution>(), new SBXCrossover(0.9, 20.0), uniformMutation);
      ibea.calculateFitness(sameRankArchive);
      // Find the best
      double best = (double) ibea.solutionFitness.getAttribute(sameRankArchive.get(0));
      int bestIndex = 0;
      for (int index = 1; index < sameRankArchive.size(); index++) {
        if ((double) ibea.solutionFitness.getAttribute(sameRankArchive.get(index)) < best) {
          best = (double) ibea.solutionFitness.getAttribute(sameRankArchive.get(index));
          bestIndex = index;
        }
      }
      // gbest <= best
      bestGlobal = sameRankArchive.get(bestIndex);
    }

    // Update using gBest
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

  // (b-3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
  protected void updateVelocityUsingNoGlobalBest(
      double W, double C1, double C2, int i, DoubleSolution particle, DoubleSolution bestParticle )
  {
    double r1, r2;
    for (int var = 0; var < particle.getNumberOfVariables(); var++) {
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      //Computing the velocity of this particle
      speed[i][var] =
          W * speed[i][var]
              + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
              + C2 * r2 * speed[i][var];
    }
  }

  @Override
  protected void initializeLeader(List<DoubleSolution> swarm) {
    // picup non-dominated solutions
    temporaryArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
    for (DoubleSolution particle : swarm) {
      temporaryArchive.add( (DoubleSolution) particle.copy() );
    }
    // add non-dominated solutions to archives
    for (DoubleSolution particle : temporaryArchive.getSolutionList()){
      truncatedArchive.add( (DoubleSolution) particle.copy() );
      epsilonArchive.add( (DoubleSolution) particle.copy() );
    }
    strengthRawFitness.computeDensityEstimator(truncatedArchive);
    truncatedArchive = environmentalSelection.execute(truncatedArchive);
  }

  /**
   * Update leaders method
   * @param swarm List of solutions (swarm)
   */
  @Override protected void updateLeaders(List<DoubleSolution> swarm) {
    temporaryArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
    for (int s=0; s<truncatedArchive.size(); s++){
      temporaryArchive.add( (DoubleSolution) truncatedArchive.get(s).copy() );
    }
    for (DoubleSolution particle : swarm) {
      temporaryArchive.add((DoubleSolution) particle.copy());
      epsilonArchive.add((DoubleSolution) particle.copy());
    }
    List<DoubleSolution> union = temporaryArchive.getSolutionList();
    if(union.size()<=1) {
      truncatedArchive.clear();
      for ( DoubleSolution particle: union){
        truncatedArchive.add((DoubleSolution) particle.copy());
      }
    }else{
      strengthRawFitness.computeDensityEstimator(union);
      truncatedArchive = environmentalSelection.execute(union);
    }
  }

  @Override public List<DoubleSolution> getResult() {
    return this.truncatedArchive;
  }

  @Override public String getName() {
    return "OMOPSODBTIBG" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament and Indicator Based gBest" ;
  }

}
