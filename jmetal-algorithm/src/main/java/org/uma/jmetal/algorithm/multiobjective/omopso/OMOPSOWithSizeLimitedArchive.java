package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.impl.AbstractParticleSwarmOptimization;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.ParticleSwarmSolution;
import org.uma.jmetal.util.archive.impl.CrowdingDistanceArchive;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.CrowdingDistanceComparator;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.solutionattribute.impl.CrowdingDistance;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSOWithSizeLimitedArchive extends OMOPSO {

  public double eta;
  public NonDominatedSolutionListArchive<DoubleSolution> temporaryArchive; // 一時アーカイブ
  public List<DoubleSolution> truncatedArchive; // SPEA2の端切りアーカイブ
  public EnvironmentalSelection<DoubleSolution> environmentalSelection; // SPEA2の環境選択
  public StrengthRawFitness<DoubleSolution> strengthRawFitness; // SPEA2の適合度

  /** Constructor */
  public OMOPSOWithSizeLimitedArchive(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                                      int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                                      NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
    this.eta = eta;
    truncatedArchive = new ArrayList<>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<DoubleSolution>();
    environmentalSelection = new EnvironmentalSelection<DoubleSolution>(archiveSize);
  }

  @Override protected void initProgress() {
    currentIteration = 1;
    crowdingDistance.computeDensityEstimator(leaderArchive.getSolutionList());
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
    dump(getSwarm(), "swarm");
    // pBestの保存
    List<DoubleSolution> bestParticle = new ArrayList<>();
    for(DoubleSolution pbest : localBest) bestParticle.add(pbest);
    dump(bestParticle, "pBest");  }

  @Override protected void updateProgress() {
    currentIteration += 1;
    crowdingDistance.computeDensityEstimator(leaderArchive.getSolutionList());
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
    dump(getSwarm(), "swarm");
    // pBestの保存
    List<DoubleSolution> bestParticle = new ArrayList<>();
    for(DoubleSolution pbest : localBest) bestParticle.add(pbest);
    dump(bestParticle, "pBest");  }

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

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;
    DoubleSolution bestGlobal;

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];

      //Select a global localBest for calculate the speed of particle i, bestGlobal
      DoubleSolution one ;
      DoubleSolution two;
      int pos1 = randomGenerator.nextInt(0, truncatedArchive.size() - 1);
      int pos2 = randomGenerator.nextInt(0, truncatedArchive.size() - 1);
      one = truncatedArchive.get(pos1);
      two = truncatedArchive.get(pos2);

      if (crowdingDistanceComparator.compare(one, two) < 1) {
        bestGlobal = one ;
      } else {
        bestGlobal = two ;
      }

      //Parameters for velocity equation
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);
      //

      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        //Computing the velocity of this particle
        speed[i][var] = W * speed[i][var] + C1 * r1 * (bestParticle.getVariableValue(var) -
            particle.getVariableValue(var)) +
            C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
      }
    }
  }

  @Override public List<DoubleSolution> getResult() {
      return this.truncatedArchive;
  }

  /**
   * Update leaders method
   * @param swarm List of solutions (swarm)
   */
  @Override protected void updateLeaders(List<DoubleSolution> swarm) {
    temporaryArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(this.eta));
    for (DoubleSolution solution : truncatedArchive){
      temporaryArchive.add((DoubleSolution) solution.copy());
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

  @Override public String getName() {
    return "OMOPSOWithSizeLimitedArchive" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO with size limited archive" ;
  }

}
