package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.solution.impl.ParticleSwarmSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.neighborhood.impl.KNearestNeighborhood;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class DirectiveOMOPSO extends OMOPSOWithSizeLimitedArchive {

  public NonDominatedSolutionListArchive<ParticleSwarmSolution> temporaryArchive; // 一時アーカイブ
  public List<ParticleSwarmSolution> truncatedArchive; // SPEA2の端切りアーカイブ
  public EnvironmentalSelection<ParticleSwarmSolution> environmentalSelection; // SPEA2の環境選択
  public StrengthRawFitness<ParticleSwarmSolution> strengthRawFitness; // SPEA2の適合度

  /** Constructor */
  public DirectiveOMOPSO(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                         int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                         NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);

    truncatedArchive = new ArrayList<ParticleSwarmSolution>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<ParticleSwarmSolution>();
    environmentalSelection = new EnvironmentalSelection<ParticleSwarmSolution>(archiveSize);
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
    ParticleSwarmSolution bestGlobal;

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
      List<ParticleSwarmSolution> dominanceArchive = new ArrayList<>(archiveSize);
      List<ParticleSwarmSolution> sameRankArchive = new ArrayList<>(archiveSize);
      for(int a=0; a < truncatedArchive.size(); a++ ){
        DominanceComparator<ParticleSwarmSolution> comparator = new DominanceComparator<ParticleSwarmSolution>();
        int dominated = comparator.compare( particle, truncatedArchive.get(a));
        // if particle is dominated by archive
        if( dominated==1 ){
          dominanceArchive.add(truncatedArchive.get(a).copy());
          // if particle is same rank of archive
        }else if(dominated ==0){
          sameRankArchive.add(truncatedArchive.get(a).copy());
        }
      }
      // (1) もし対象particleよりもarchiveに優越している個体があれば，その個体からランダムでglobalbestを決定する．
      if(dominanceArchive.size() >0 ) {
        int pos = randomGenerator.nextInt(0, dominanceArchive.size() - 1);
        bestGlobal = dominanceArchive.get(pos);
        for (int var = 0; var < particle.getNumberOfVariables(); var++) {
          //Computing the velocity of this particle
          speed[i][var] =
              W * speed[i][var]
            + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
            + C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
        }
      // (2) 対象particleとarchiveが同一ランクであれば，同一ランクarchiveから近い10個体のうちランダムでvelocityを足し合わせる
      }else if(sameRankArchive.size() > 0){
        sameRankArchive.add(particle);
        int maxNeighborSize = 10;
        int neighborSize = sameRankArchive.size()<maxNeighborSize ? sameRankArchive.size()-1 :  maxNeighborSize-1;
        KNearestNeighborhood<ParticleSwarmSolution> neighborhood = new KNearestNeighborhood<ParticleSwarmSolution>( neighborSize );
        List<ParticleSwarmSolution> neighbors = neighborhood.getNeighbors(sameRankArchive, sameRankArchive.size()-1);
        int pos = randomGenerator.nextInt(0, neighbors.size() - 1);
        bestGlobal = neighbors.get(pos);
        for (int var = 0; var < particle.getNumberOfVariables(); var++) {
          //Computing the velocity of this particle
          speed[i][var] =
              W * speed[i][var]
            + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
            + C2 * r2 * bestGlobal.getSpeed(var);
        }
      }
      // (3) アーカイブ全てが対象particleに優越されていたら，globalBestは用いないで飛翔する．
      else{
        for (int var = 0; var < particle.getNumberOfVariables(); var++) {
          //Computing the velocity of this particle
          speed[i][var] =
              W * speed[i][var]
            + C1 * r1 * (bestParticle.getVariableValue(var) - particle.getVariableValue(var))
            + C2 * r2 * speed[i][var];
        }
      }
    }
  }

  protected double updateVelocityUsingDominanceArchive(double currentSpeed, List<DoubleSolution> archive) {
    return 0;
  }

  protected double updateVelocityUsingSameRankArchive(List<DoubleSolution> archive) {
    return 0;
  }

  protected double updateVelocityItself(double currentSpeed) {
    return 0;
  }

  @Override
  protected void initializeLeader(List<DoubleSolution> swarm) {
    for (int s=0; s<swarm.size(); s++) {
      // make solution include speed.
      ParticleSwarmSolution particle = new ParticleSwarmSolution(swarm.get(s));

      truncatedArchive.add( particle.copy() );
      epsilonArchive.add( particle.copy() );
    }
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
    return "DirectionalOMOPSO" ;
  }

  @Override public String getDescription() {
    return "Directional optimized MOPSO with size limited archive" ;
  }

}
