package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.algorithm.multiobjective.ibea.IBEA;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.List;

/** Class implementing the OMOPSO algorithm with archive using truncation method */
@SuppressWarnings("serial")
public class OMOPSORVIBP extends OMOPSORV {

  /** Constructor */
  public OMOPSORVIBP(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                     int swarmSize, int maxIterations, int leaderSize, UniformMutation uniformMutation,
                     NonUniformMutation nonUniformMutation, double eta) {
    super(problem, evaluator, swarmSize, maxIterations, leaderSize, uniformMutation, nonUniformMutation, eta);
  }

  // [IBP] pBest更新が同一ランク解で更新される場合，IBEAのcalculateFitnessの結果が良ければ更新するが，悪ければ更新しない．
  private void updateParticlesMemoryBasedIndicator(List<DoubleSolution> swarm, int i)
  {
    List<DoubleSolution> temporaryArchive = new ArrayList<DoubleSolution>();
    temporaryArchive.add((DoubleSolution)swarm.get(i).copy());
    temporaryArchive.add((DoubleSolution)localBest[i].copy());

    // IBEAをインスタンス化してcalculateFitnessをする
    IBEA<DoubleSolution> ibea = new IBEA<DoubleSolution>(problem, swarmSize, archiveSize, maxIterations,
        new BinaryTournamentSelection<DoubleSolution>(), new SBXCrossover(0.9, 20.0), uniformMutation);
    ibea.calculateFitness(temporaryArchive);
    double fitnessOfCurrentParticle = (double)ibea.solutionFitness.getAttribute(temporaryArchive.get(0));
    double fitnessOfLocalBest = (double)ibea.solutionFitness.getAttribute(temporaryArchive.get(1));
    if(fitnessOfLocalBest < fitnessOfCurrentParticle){
      localBest[i] = temporaryArchive.get(1);
    }
  }

  @Override
  protected void updateParticlesMemory(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      int flag = dominanceComparator.compare(swarm.get(i), localBest[i]);
      if(flag == 0){
        updateParticlesMemoryBasedIndicator(swarm, i);
      }else if(flag== -1) {
        DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
        localBest[i] = particle;
      }
    }
  }

  @Override public String getName() {
    return "OMOPSORVIBP" ;
  }

  @Override public String getDescription() {
    return "Optimized MOPSO with Dominated Binary Tournament, Indicator Based Gbest and indicator based pBest" ;
  }

}
