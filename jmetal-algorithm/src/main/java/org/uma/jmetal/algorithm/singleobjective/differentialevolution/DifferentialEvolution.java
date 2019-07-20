package org.uma.jmetal.algorithm.singleobjective.differentialevolution;

import org.uma.jmetal.algorithm.impl.AbstractDifferentialEvolution;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.impl.selection.DifferentialEvolutionSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.comparator.ObjectiveComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * This class implements a differential evolution algorithm.
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
@SuppressWarnings("serial")
public class DifferentialEvolution extends AbstractDifferentialEvolution<DoubleSolution> {
  private int populationSize;
  private int maxEvaluations;
  private SolutionListEvaluator<DoubleSolution> evaluator;
  private Comparator<DoubleSolution> comparator;

  private int evaluations;
  private int currentGeneration;
  protected List<DoubleSolution> initialPopulation;

  /**
   * Constructor
   *
   * @param problem Problem to solve
   * @param maxEvaluations Maximum number of evaluations to perform
   * @param populationSize
   * @param crossoverOperator
   * @param selectionOperator
   * @param evaluator
   */
  public DifferentialEvolution(DoubleProblem problem, int maxEvaluations, int populationSize,
      DifferentialEvolutionCrossover crossoverOperator,
      DifferentialEvolutionSelection selectionOperator, SolutionListEvaluator<DoubleSolution> evaluator) {
    setProblem(problem);
    this.maxEvaluations = maxEvaluations;
    this.populationSize = populationSize;
    this.crossoverOperator = crossoverOperator;
    this.selectionOperator = selectionOperator;
    this.evaluator = evaluator;

    comparator = new ObjectiveComparator<DoubleSolution>(0);
    this.initialPopulation = null;
  }

  public DifferentialEvolution(DoubleProblem problem, int maxEvaluations, int populationSize,
                               DifferentialEvolutionCrossover crossoverOperator,
                               DifferentialEvolutionSelection selectionOperator,
                               SolutionListEvaluator<DoubleSolution> evaluator,
                               Comparator<DoubleSolution> comparator) {
    setProblem(problem);
    this.maxEvaluations = maxEvaluations;
    this.populationSize = populationSize;
    this.crossoverOperator = crossoverOperator;
    this.selectionOperator = selectionOperator;
    this.evaluator = evaluator;

    this.comparator = comparator;
    this.initialPopulation = null;
  }


  public int getEvaluations() {
    return evaluations;
  }

  public void setEvaluations(int evaluations) {
    this.evaluations = evaluations;
  }

  @Override protected void initProgress() {
    evaluations = populationSize;
    currentGeneration = 1;
    dump();
  }

  @Override protected void updateProgress() {
    evaluations += populationSize;
    currentGeneration += 1;
    dump();
  }

  protected void dump(){
    // dump solution list in the searching
    List<DoubleSolution> population = getPopulation();
    new SolutionListOutput(population)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/variable" + currentGeneration + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/fitness" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<DoubleSolution>(population)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/constraint" + currentGeneration + ".csv"))
            .setSeparator(",")
            .print();
  }

  @Override protected boolean isStoppingConditionReached() {
    return evaluations >= maxEvaluations;
  }

  public void setInitialPopulation(List<DoubleSolution> initialPopulation) {
    this.initialPopulation = initialPopulation;
  }
  @Override
  public List<DoubleSolution> createInitialPopulation(){
    List<DoubleSolution> population = new ArrayList<>(populationSize);
    if(this.initialPopulation == null){
      for (int i = 0; i < populationSize; i++) {
        DoubleSolution newIndividual = getProblem().createSolution();
        population.add(newIndividual);
      }
    }else{
      population = this.initialPopulation;
    }
    return population;
  }

  @Override protected List<DoubleSolution> evaluatePopulation(List<DoubleSolution> population) {
    return evaluator.evaluate(population, getProblem());
  }

  @Override protected List<DoubleSolution> selection(List<DoubleSolution> population) {
    return population;
  }

  @Override protected List<DoubleSolution> reproduction(List<DoubleSolution> matingPopulation) {
    List<DoubleSolution> offspringPopulation = new ArrayList<>();

    for (int i = 0; i < populationSize; i++) {
      selectionOperator.setIndex(i);
      List<DoubleSolution> parents = selectionOperator.execute(matingPopulation);

      crossoverOperator.setCurrentSolution(matingPopulation.get(i));
      List<DoubleSolution> children = crossoverOperator.execute(parents);

      offspringPopulation.add(children.get(0));
    }

    return offspringPopulation;
  }

  @Override protected List<DoubleSolution> replacement(List<DoubleSolution> population,
      List<DoubleSolution> offspringPopulation) {
    List<DoubleSolution> pop = new ArrayList<>();

    for (int i = 0; i < populationSize; i++) {
      if (comparator.compare(population.get(i), offspringPopulation.get(i)) < 0) {
        pop.add(population.get(i));
      } else {
        pop.add(offspringPopulation.get(i));
      }
    }

    Collections.sort(pop, comparator) ;
    return pop;
  }

  /**
   * Returns the best individual
   */
  @Override public DoubleSolution getResult() {
    Collections.sort(getPopulation(), comparator) ;

    return getPopulation().get(0);
  }

  @Override public String getName() {
    return "DE" ;
  }

  @Override public String getDescription() {
    return "Differential Evolution Algorithm" ;
  }
}
