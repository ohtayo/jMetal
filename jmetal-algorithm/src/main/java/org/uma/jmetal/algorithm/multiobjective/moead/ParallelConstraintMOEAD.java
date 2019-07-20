package org.uma.jmetal.algorithm.multiobjective.moead;

import org.uma.jmetal.algorithm.multiobjective.moead.util.MOEADUtils;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.comparator.impl.ViolationThresholdComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import java.util.ArrayList;
import java.util.List;

/**
 * This class implements a parallel version of the MOEAD algorithm.

 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ParallelConstraintMOEAD extends AbstractMOEAD<DoubleSolution>  {

  private DifferentialEvolutionCrossover differentialEvolutionCrossover ;
  private ViolationThresholdComparator<DoubleSolution> violationThresholdComparator ;
  private SolutionListEvaluator<DoubleSolution> evaluator;
  private List<DoubleSolution> initialPopulation;

  public ParallelConstraintMOEAD(Problem<DoubleSolution> problem,
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
    super(problem, populationSize, resultPopulationSize, maxEvaluations, crossover, mutation, functionType,
        dataDirectory, neighborhoodSelectionProbability, maximumNumberOfReplacedSolutions,
        neighborSize);

    differentialEvolutionCrossover = (DifferentialEvolutionCrossover)crossoverOperator ;
    violationThresholdComparator = new ViolationThresholdComparator<DoubleSolution>() ;
    this.evaluator = evaluator;
    this.initialPopulation = null;
  }

  @Override public void run() {
    initializeUniformWeight();
    initializeNeighborhood();
    initializePopulation();
    idealPoint.update(population);
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
      dump();

    } while (evaluations < maxEvaluations);
  }

  // 並列評価
  protected List<DoubleSolution> evaluate(List<DoubleSolution> children) {
    children = evaluator.evaluate(children, problem);

    return children;
  }

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
  }

  public void setInitialPopulation(List<DoubleSolution> initialPopulation) {
    this.initialPopulation = initialPopulation;
  }
  public void initializePopulation() {
    if(this.initialPopulation==null) {
      for (int i = 0; i < populationSize; i++) {
        DoubleSolution newSolution = (DoubleSolution) problem.createSolution();
        population.add(newSolution);
      }
    }else{
      population = this.initialPopulation;
    }
    population = evaluate(population);
  }

  @Override
  protected void updateNeighborhood(DoubleSolution individual, int subproblemId, NeighborType neighborType) {
    int size;
    int time;

    time = 0;

    if (neighborType == NeighborType.NEIGHBOR) {
      size = neighborhood[subproblemId].length;
    } else {
      size = population.size();
    }
    int[] perm = new int[size];

    MOEADUtils.randomPermutation(perm, size);

    for (int i = 0; i < size; i++) {
      int k;
      if (neighborType == NeighborType.NEIGHBOR) {
        k = neighborhood[subproblemId][perm[i]];
      } else {
        k = perm[i];
      }
      double f1, f2;

      f1 = fitnessFunction(population.get(k), lambda[k]);
      f2 = fitnessFunction(individual, lambda[k]);

      if (violationThresholdComparator.needToCompare(population.get(k), individual)) {
        int flag = violationThresholdComparator.compare(population.get(k), individual);
        if (flag == 1) {
          population.set(k, (DoubleSolution) individual.copy());
        } else if (flag == 0) {
          if (f2 < f1) {
            population.set(k, (DoubleSolution) individual.copy());
            time++;
          }
        }
      } else {
        if (f2 < f1) {
          population.set(k, (DoubleSolution) individual.copy());
          time++;
        }
      }

      if (time >= maximumNumberOfReplacedSolutions) {
        return;
      }
    }
  }

  @Override public String getName() {
    return "ParallelConstraintMOEAD" ;
  }

  @Override public String getDescription() {
    return "Parallel version of Multi-Objective Evolutionary Algorithm based on Decomposition with constraints support" ;
  }
}
