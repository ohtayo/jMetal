package org.uma.jmetal.algorithm.multiobjective.moead;

import org.uma.jmetal.algorithm.multiobjective.moead.util.MOEADUtils;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.impl.ViolationThresholdComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

/**
 * This class implements a parallel version of the MOEA/D-DE algorithm.

 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ParallelConstraintMOEADWithEpsilonArchive extends ParallelConstraintMOEAD  {
  private double eta = 0.0075;
  private NonDominatedSolutionListArchive<DoubleSolution> epsilonArchive;
  private NonDominatedSolutionListArchive<DoubleSolution> temporaryArchive;
  private List<DoubleSolution> truncatedArchive;
  private StrengthRawFitness<DoubleSolution> strengthRawFitness;
  private EnvironmentalSelection<DoubleSolution> environmentalSelection;

  public ParallelConstraintMOEADWithEpsilonArchive(Problem<DoubleSolution> problem,
                                                   int populationSize,
                                                   int resultPopulationSize,
                                                   int archiveSize,
                                                   int maxEvaluations,
                                                   MutationOperator<DoubleSolution> mutation,
                                                   CrossoverOperator<DoubleSolution> crossover,
                                                   FunctionType functionType,
                                                   String dataDirectory,
                                                   double neighborhoodSelectionProbability,
                                                   int maximumNumberOfReplacedSolutions,
                                                   int neighborSize,
                                                   SolutionListEvaluator<DoubleSolution> evaluator) {
    super(problem, populationSize, resultPopulationSize, maxEvaluations, mutation, crossover,functionType,
        dataDirectory, neighborhoodSelectionProbability, maximumNumberOfReplacedSolutions,
        neighborSize, evaluator);
    epsilonArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(eta));
    truncatedArchive = new ArrayList<>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<DoubleSolution>();
    environmentalSelection = new EnvironmentalSelection<DoubleSolution>(archiveSize);
  }

  public void setEta(double eta) {
    this.eta = eta ;
    epsilonArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(eta));
  }

  public double getEta(){
    return eta;
  }

  protected void updateArchive(List<DoubleSolution> population)
  {
    temporaryArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(eta));
    for (DoubleSolution solution : truncatedArchive){
      temporaryArchive.add((DoubleSolution) solution.copy());
    }
    for (DoubleSolution solution : population){
      temporaryArchive.add((DoubleSolution) solution.copy());
      epsilonArchive.add((DoubleSolution) solution.copy());
    }
    List<DoubleSolution> union = temporaryArchive.getSolutionList();
    strengthRawFitness.computeDensityEstimator(union);
    truncatedArchive = environmentalSelection.execute(union);
  }

  @Override public void run() {
    initializeUniformWeight();
    initializeNeighborhood();
    initializePopulation();
    idealPoint.update(population);
    updateArchive(population);

    violationThresholdComparator.updateThreshold(population);

    evaluations = populationSize ;
    dump(getResult());
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");

    do {
      int[] permutation = new int[populationSize];
      MOEADUtils.randomPermutation(permutation, populationSize);

      List<DoubleSolution> childrenPool = new ArrayList<DoubleSolution>();  // 生成した子の格納場所を確保
      List<Integer> subProblemIds = new ArrayList<Integer>();
      List<NeighborType> neighborTypes = new ArrayList<NeighborType>();

      for (int i = 0; i < populationSize; i++) {
        int subProblemId = permutation[i];
        subProblemIds.add(subProblemId);

        NeighborType neighborType = chooseNeighborType();
        neighborTypes.add(neighborType);
        List<DoubleSolution> parents = parentSelection(subProblemId, neighborType); // MOEA/Dは3つの親を生成する．SBXは2つ親を必要とするので要素を1つ削除する．
        parents.remove(parents.size()-1); // 末尾の要素を削除

        List<DoubleSolution> children = sbxCrossover.execute(parents);

        DoubleSolution child = children.get(0);
        mutationOperator.execute(child);
        // 子を格納
        childrenPool.add(child);
      }

      childrenPool = evaluate(childrenPool);
      evaluations+=childrenPool.size();

      for(int i=0; i< childrenPool.size(); i++){
        DoubleSolution child = childrenPool.get(i);
        idealPoint.update(child.getObjectives());
        updateNeighborhood(child, subProblemIds.get(i), neighborTypes.get(i));
      }

      violationThresholdComparator.updateThreshold(population);
      updateArchive(childrenPool);
      dump(getResult());
      dump(epsilonArchive.getSolutionList(), "epsilon");
      dump(truncatedArchive, "truncated");

    } while (evaluations < maxEvaluations);
  }

  @Override public String getName() {
    return "ParallelConstraintMOEADWithEpsilonArchive" ;
  }

  @Override public String getDescription() {
    return "Parallel version of Multi-Objective Evolutionary Algorithm based on Decomposition with epsilon archive and constraints support." ;
  }
}
