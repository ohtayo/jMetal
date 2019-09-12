package org.uma.jmetal.algorithm.multiobjective.nsgaii;

import org.uma.jmetal.algorithm.impl.AbstractGeneticAlgorithm;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.selection.RankingAndCrowdingSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
@SuppressWarnings("serial")
public class NSGAIIWithEpsilonArchive<S extends Solution<?>> extends NSGAII<S> {
  protected final SolutionListEvaluator<S> evaluator;
  private int currentGeneration;
  private NonDominatedSolutionListArchive<S> epsilonArchive;
  private double eta = 0.0075;

  /**
   * Constructor
   */
  public NSGAIIWithEpsilonArchive(Problem<S> problem, int maxEvaluations, int populationSize,
                                  int matingPoolSize, int offspringPopulationSize,
                                  CrossoverOperator<S> crossoverOperator, MutationOperator<S> mutationOperator,
                                  SelectionOperator<List<S>, S> selectionOperator, SolutionListEvaluator<S> evaluator) {
    this(problem, maxEvaluations, populationSize, matingPoolSize, offspringPopulationSize,
            crossoverOperator, mutationOperator, selectionOperator, new DominanceComparator<S>(), evaluator);
  }
  /**
   * Constructor
   */
  public NSGAIIWithEpsilonArchive(Problem<S> problem, int maxEvaluations, int populationSize,
                                  int matingPoolSize, int offspringPopulationSize,
                                  CrossoverOperator<S> crossoverOperator, MutationOperator<S> mutationOperator,
                                  SelectionOperator<List<S>, S> selectionOperator, Comparator<S> dominanceComparator,
                                  SolutionListEvaluator<S> evaluator) {
    super(problem, maxEvaluations, populationSize, matingPoolSize, offspringPopulationSize,
            crossoverOperator, mutationOperator, selectionOperator, dominanceComparator, evaluator) ;
    setMaxPopulationSize(populationSize); ;

    this.evaluator = evaluator;
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
  }
  public void setEta(double eta) {
    this.eta = eta ;
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
  }
  public double getEta(){
    return eta;
  }

  public void setInitialPopulation(List<S> initialPopulation) {
    this.initialPopulation = initialPopulation;
  }
  @Override
  public List<S> createInitialPopulation(){
    List<S> population = new ArrayList<>(getMaxPopulationSize());
    if(this.initialPopulation == null){
      for (int i = 0; i < getMaxPopulationSize(); i++) {
        S newIndividual = getProblem().createSolution();
        population.add(newIndividual);
      }
    }else{
      population = this.initialPopulation;
    }
    return population;
  }

  @Override protected void initProgress() {
    currentGeneration = 1;
    evaluations = getMaxPopulationSize();
    updateArchive(population);
    dump();
  }

  @Override protected void updateProgress() {
    currentGeneration += 1;
    evaluations += offspringPopulationSize;
    updateArchive(population);
    dump();
  }

  protected void dump(){
    List<S> population = getResult();
    new SolutionListOutput(population)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/variable" + currentGeneration + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/fitness" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<S>(population)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/constraint" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
    List<S> epsilon = epsilonArchive.getSolutionList();
    new SolutionListOutput(epsilon)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/epsilonVariable" + currentGeneration + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/epsilonFitness" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<S>(epsilon)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/epsilonConstraint" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
  }
  protected void updateArchive(List<S> population)
  {
    for (S solution : population){
      epsilonArchive.add((S) solution.copy());
    }
  }

  @Override protected boolean isStoppingConditionReached() {
    return evaluations >= maxEvaluations;
  }

  @Override protected List<S> evaluatePopulation(List<S> population) {
    population = evaluator.evaluate(population, getProblem());

    return population;
  }

  /**
   * This method iteratively applies a {@link SelectionOperator} to the population to fill the mating pool population.
   *
   * @param population
   * @return The mating pool population
   */
  @Override
  protected List<S> selection(List<S> population) {
    List<S> matingPopulation = new ArrayList<>(population.size());
    for (int i = 0; i < matingPoolSize; i++) {
      S solution = selectionOperator.execute(population);
      matingPopulation.add(solution);
    }

    return matingPopulation;
  }

  /**
   * This methods iteratively applies a {@link CrossoverOperator} a  {@link MutationOperator} to the population to
   * create the offspring population. The population size must be divisible by the number of parents required
   * by the {@link CrossoverOperator}; this way, the needed parents are taken sequentially from the population.
   *
   * The number of solutions returned by the {@link CrossoverOperator} must be equal to the offspringPopulationSize
   * state variable
   *
   * @param matingPool
   * @return The new created offspring population
   */
  @Override
  protected List<S> reproduction(List<S> matingPool) {
    int numberOfParents = crossoverOperator.getNumberOfRequiredParents() ;

    checkNumberOfParents(matingPool, numberOfParents);

    List<S> offspringPopulation = new ArrayList<>(offspringPopulationSize);
    for (int i = 0; i < matingPool.size(); i += numberOfParents) {
      List<S> parents = new ArrayList<>(numberOfParents);
      for (int j = 0; j < numberOfParents; j++) {
        parents.add(population.get(i+j));
      }

      List<S> offspring = crossoverOperator.execute(parents);

      for(S s: offspring){
        mutationOperator.execute(s);
        offspringPopulation.add(s);
        if (offspringPopulation.size() >= offspringPopulationSize)
          break;
      }
    }
    return offspringPopulation;
  }

  @Override protected List<S> replacement(List<S> population, List<S> offspringPopulation) {
    List<S> jointPopulation = new ArrayList<>();
    jointPopulation.addAll(population);
    jointPopulation.addAll(offspringPopulation);

    RankingAndCrowdingSelection<S> rankingAndCrowdingSelection ;
    rankingAndCrowdingSelection = new RankingAndCrowdingSelection<S>(getMaxPopulationSize(), dominanceComparator) ;

    return rankingAndCrowdingSelection.execute(jointPopulation) ;
  }

  @Override public List<S> getResult() {
    return SolutionListUtils.getNondominatedSolutions(getPopulation());
  }

  @Override public String getName() {
    return "NSGAII" ;
  }

  @Override public String getDescription() {
    return "Nondominated Sorting Genetic Algorithm version II" ;
  }
}
