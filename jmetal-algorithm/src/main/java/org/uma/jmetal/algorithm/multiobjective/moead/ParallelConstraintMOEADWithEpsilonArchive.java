package org.uma.jmetal.algorithm.multiobjective.moead;

import org.uma.jmetal.algorithm.multiobjective.moead.util.MOEADUtils;
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

import javax.swing.*;
import java.util.ArrayList;
import java.util.List;

/**
 * This class implements a parallel version of the MOEAD algorithm.

 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ParallelConstraintMOEADWithEpsilonArchive extends ParallelConstraintMOEAD  {
  private NonDominatedSolutionListArchive<DoubleSolution> epsilonArchive;
  private double eta = 0.0075;

  public ParallelConstraintMOEADWithEpsilonArchive(Problem<DoubleSolution> problem,
                                                   int populationSize,
                                                   int resultPopulationSize,
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
    for (DoubleSolution solution : population){
      epsilonArchive.add((DoubleSolution) solution.copy());
    }
  }
  @Override public void run() {
    initializeUniformWeight();
    initializeNeighborhood();
    initializePopulation();
    idealPoint.update(population);
    updateArchive(population);
    dump();

    violationThresholdComparator.updateThreshold(population);

    evaluations = populationSize ;

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
        List<DoubleSolution> parents = parentSelection(subProblemId, neighborType);

        differentialEvolutionCrossover.setCurrentSolution(population.get(subProblemId));
        List<DoubleSolution> children = differentialEvolutionCrossover.execute(parents);

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
      dump();

    } while (evaluations < maxEvaluations);
  }

  @Override
  protected void dump()
  {
    // dump solution list in the searching
    List<DoubleSolution> result = getResult();
    new SolutionListOutput(result)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/variable" + Integer.valueOf(evaluations/populationSize) + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/fitness" + Integer.valueOf(evaluations/populationSize) + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<DoubleSolution>(result)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/constraint" + Integer.valueOf(evaluations/populationSize)  + ".csv"))
            .setSeparator(",")
            .print();
    List<DoubleSolution> epsilon = epsilonArchive.getSolutionList();
    new SolutionListOutput(epsilon)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/epsilonVariable" + Integer.valueOf(evaluations/populationSize) + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/epsilonFitness" + Integer.valueOf(evaluations/populationSize) + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<DoubleSolution>(epsilon)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/epsilonConstraint" + Integer.valueOf(evaluations/populationSize) + ".csv"))
            .setSeparator(",")
            .print();
  }

  @Override public String getName() {
    return "ParallelConstraintMOEADWithEpsilonArchive" ;
  }

  @Override public String getDescription() {
    return "Parallel version of Multi-Objective Evolutionary Algorithm based on Decomposition with epsilon archive and constraints support." ;
  }
}
