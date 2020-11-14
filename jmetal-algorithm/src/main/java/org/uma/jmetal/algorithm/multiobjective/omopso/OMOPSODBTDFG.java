package org.uma.jmetal.algorithm.multiobjective.omopso;

import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.solution.impl.ParticleSwarmSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.StrengthFitnessComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.neighborhood.impl.KNearestNeighborhood;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSODBTDFG extends OMOPSOWithSizeLimitedArchive {

  public NonDominatedSolutionListArchive<ParticleSwarmSolution> temporaryArchive; // 一時アーカイブ
  public List<ParticleSwarmSolution> truncatedArchive; // SPEA2の端切りアーカイブ
  public EnvironmentalSelection<ParticleSwarmSolution> environmentalSelection; // SPEA2の環境選択
  public StrengthRawFitness<ParticleSwarmSolution> strengthRawFitness; // SPEA2の適合度
  private StrengthFitnessComparator<ParticleSwarmSolution> strengthFitnessComparator;
  /** Constructor */
  public OMOPSODBTDFG(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                      int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                      NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);

    truncatedArchive = new ArrayList<ParticleSwarmSolution>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<ParticleSwarmSolution>();
    environmentalSelection = new EnvironmentalSelection<ParticleSwarmSolution>(archiveSize);
    strengthFitnessComparator = new StrengthFitnessComparator<ParticleSwarmSolution>();
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

      // 飛翔させる対象粒子とアーカイブの優越関係を確認
      NonDominatedSolutionListArchive<ParticleSwarmSolution> dominanceArchive = new NonDominatedSolutionListArchive<ParticleSwarmSolution>(new DominanceComparator<ParticleSwarmSolution>(this.eta));
      List<ParticleSwarmSolution> sameRankArchive = new ArrayList<>(archiveSize);
      for (int a = 0; a < truncatedArchive.size(); a++) {
        DominanceComparator<ParticleSwarmSolution> comparator = new DominanceComparator<ParticleSwarmSolution>();
        int dominated = comparator.compare(particle, truncatedArchive.get(a));
        // if particle is dominated by archive
        if (dominated == 1) {
          dominanceArchive.add(truncatedArchive.get(a).copy());
          // if particle is same rank of archive
        } else if (dominated == 0) {
          sameRankArchive.add(truncatedArchive.get(a).copy());
        }
      }
      // (1)[DBT] もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
      if (dominanceArchive.size() > 0) {
        updateVelocityUsingDominanceArchive(W, C1, C2, r1, r2, i, particle, dominanceArchive.getSolutionList(), bestParticle);
      }
      // (2)[DFG] 対象particleとarchiveが同一ランクであれば，同一ランクarchiveから近い10個体のうちランダムでvelocityを足し合わせる
      else if (sameRankArchive.size() > 0) {
        updateVelocityUsingSameRankArchivePosition(W, C1, C2, r1, r2, i, particle, sameRankArchive);  // (b-2-II)の手法を用いる
      }
      // (3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
      else {
        updateVelocityUsingNoGlobalBest(W, C1, C2, r1, r2, i, particle, bestParticle);
      }
    }
  }

  // (b-1) もし対象particleよりもarchiveに優越している個体があれば，その個体からバイナリトーナメント選択でglobalbestを決定する．
  protected void updateVelocityUsingDominanceArchive(
      double W, double C1, double C2, double r1, double r2, int i, ParticleSwarmSolution particle, List<ParticleSwarmSolution> dominanceArchive, DoubleSolution bestParticle
  ){
    if ( dominanceArchive.size()==1 ){
      dominanceArchive.add(dominanceArchive.get(0));
    }

    // random select
    ParticleSwarmSolution one ;
    ParticleSwarmSolution two;
    int pos1 = randomGenerator.nextInt(0, dominanceArchive.size() - 1);
    int pos2 = randomGenerator.nextInt(0, dominanceArchive.size() - 1);
    one = dominanceArchive.get(pos1);
    two = dominanceArchive.get(pos2);

    // binary tournament selection using strength raw fitness
    ParticleSwarmSolution bestGlobal;
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
  private boolean contains(List<ParticleSwarmSolution> list, ParticleSwarmSolution solution, int[] index){
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
  protected void updateVelocityUsingSameRankArchivePosition(
      double W, double C1, double C2, double r1, double r2, int i, ParticleSwarmSolution particle, List<ParticleSwarmSolution> sameRankArchive
  ) {
    DoubleSolution bestParticle = localBest[i];
    List<ParticleSwarmSolution> neighbors;

    if(sameRankArchive.size()==1){
      neighbors = sameRankArchive;
    }else{
      // sameRankArchiveにparticleがなければ格納，すでにある場合はそのindexを記録
      int[] index = new int[1];
      int solutionIndex=0;
      if(this.contains(sameRankArchive, particle, index)) {
        solutionIndex = index[0];
      }else{
        sameRankArchive.add(particle);
        solutionIndex = sameRankArchive.size() - 1;
      }

      // sameRankArchiveから近傍解を最大10個抽出
      int maxNeighborSize = 10;
      int neighborSize = sameRankArchive.size() <= maxNeighborSize ? sameRankArchive.size() - 1 : maxNeighborSize;  // maxNeighborSizeに満たなければすべてneighborとする
      KNearestNeighborhood<ParticleSwarmSolution> neighborhood = new KNearestNeighborhood<ParticleSwarmSolution>(neighborSize);
      neighbors = neighborhood.getNeighbors(sameRankArchive, solutionIndex);
    }

    // neighborsのvelocityとparticleのpositionを計算のために分離
    Matrix velocityOfNeighbors = new Matrix(neighbors.size(), particle.getNumberOfVariables());
    Matrix positionOfNeighbors = new Matrix(neighbors.size(), particle.getNumberOfVariables());
    for(int n=0; n<neighbors.size(); n++){
      for(int v=0; v<particle.getNumberOfVariables(); v++) {
        velocityOfNeighbors.set(n, v, neighbors.get(n).getSpeed(v));
        positionOfNeighbors.set(n, v, neighbors.get(n).getVariableValue(v));
      }
    }

    // neighborsのvelocityをparticleのpositionに足し合わせて更新する
    Matrix positionAfterFlights = positionOfNeighbors.plus(velocityOfNeighbors);
    // Todo: ここで値が一部負になった場合でも境界値まで引き戻ししなくてよいか．

    // neighborsのpositionの平均をとり，bestGlobalとする．
    Vector averagedPosition = positionAfterFlights.mean(Matrix.DIRECTION_COLUMN);
    ParticleSwarmSolution bestGlobal = neighbors.get(0);
    for(int v=0; v<particle.getNumberOfVariables(); v++){
      bestGlobal.setVariableValue(v, averagedPosition.get(v));
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

  @Override
  protected void initializeLeader(List<DoubleSolution> swarm) {
    // picup non-dominated solutions
    temporaryArchive = new NonDominatedSolutionListArchive<ParticleSwarmSolution>(new DominanceComparator<ParticleSwarmSolution>(this.eta));
    for (DoubleSolution particle : swarm) {
      temporaryArchive.add( new ParticleSwarmSolution(particle).copy() );
    }
    // add non-dominated solutions to archives
    for (ParticleSwarmSolution particle : temporaryArchive.getSolutionList()){
      truncatedArchive.add( particle.copy() );
      epsilonArchive.add( particle.copy() );
    }
    strengthRawFitness.computeDensityEstimator(truncatedArchive);
    truncatedArchive = environmentalSelection.execute(truncatedArchive);
  }

  /**
   * Update leaders method
   * @param swarm List of solutions (swarm)
   */
  @Override protected void updateLeaders(List<DoubleSolution> swarm) {
    temporaryArchive = new NonDominatedSolutionListArchive<ParticleSwarmSolution>(new DominanceComparator<ParticleSwarmSolution>(this.eta));
    for (int s=0; s<truncatedArchive.size(); s++){
      temporaryArchive.add( truncatedArchive.get(s).copy() );
    }
    for (int s=0; s<swarm.size(); s++) {
      // make solution include speed.
      ParticleSwarmSolution particle = new ParticleSwarmSolution(swarm.get(s));
      for( int v=0; v<swarm.get(s).getNumberOfVariables();  v++ ){
        particle.setSpeed(v, speed[s][v]);
      }

      temporaryArchive.add( particle.copy() );
      epsilonArchive.add( particle.copy() );
    }
    List<ParticleSwarmSolution> union = temporaryArchive.getSolutionList();
    if(union.size()<=1) {
      truncatedArchive.clear();
      for ( ParticleSwarmSolution particle: union){
        truncatedArchive.add(particle.copy());
      }
    }else{
      strengthRawFitness.computeDensityEstimator(union);
      truncatedArchive = environmentalSelection.execute(union);
    }
  }

  @Override public List<DoubleSolution> getResult() {
    List<DoubleSolution> solutions = new ArrayList<DoubleSolution>(truncatedArchive.size());
    for (ParticleSwarmSolution particle : truncatedArchive){
      solutions.add(particle.getDoubleSolution());
    }
    return solutions;
  }

  @Override public String getName() {
    return "OMOPSODBTDFG" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament and Directional Flight gBest" ;
  }

}
