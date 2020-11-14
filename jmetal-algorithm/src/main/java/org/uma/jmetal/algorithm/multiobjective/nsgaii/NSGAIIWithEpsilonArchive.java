package org.uma.jmetal.algorithm.multiobjective.nsgaii;

import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.selection.RankingAndCrowdingSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class NSGAIIWithEpsilonArchive<S extends Solution<?>> extends NSGAII<S> {
  private double eta = 0.0075;  // eta of epsilon archive
  private NonDominatedSolutionListArchive<S> epsilonArchive;  // epsilon archive
  private NonDominatedSolutionListArchive<S> temporaryArchive;  // 一時アーカイブ
  private List<S> truncatedArchive; // SPEA2の端切りアーカイブ
  private StrengthRawFitness<S> strengthRawFitness; // SPEA2の適合度
  private EnvironmentalSelection<S> environmentalSelection; // SPEA2の環境選択

  /**
   * Constructor
   */
  public NSGAIIWithEpsilonArchive(Problem<S> problem, int maxEvaluations, int populationSize,
                                  int matingPoolSize, int offspringPopulationSize, int archiveSize,
                                  CrossoverOperator<S> crossoverOperator, MutationOperator<S> mutationOperator,
                                  SelectionOperator<List<S>, S> selectionOperator, Comparator<S> dominanceComparator,
                                  SolutionListEvaluator<S> evaluator) {
    super(problem, maxEvaluations, populationSize,
            matingPoolSize, offspringPopulationSize,
            crossoverOperator, mutationOperator,
            selectionOperator, dominanceComparator,
            evaluator) ;

    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
    truncatedArchive = new ArrayList<>(archiveSize);
    strengthRawFitness = new StrengthRawFitness<S>();
    environmentalSelection = new EnvironmentalSelection<S>(archiveSize);
  }

  // setter for eta
  public void setEta(double eta) {
    this.eta = eta ;
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
  }
  // getter for eta
  public double getEta(){
    return eta;
  }

  @Override protected void initProgress() {
    currentGeneration = 1;
    evaluations = getMaxPopulationSize();
    updateArchive(population);
    dump(getResult());
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
  }

  @Override protected void updateProgress() {
    currentGeneration += 1;
    evaluations += offspringPopulationSize;
    dump(getResult());
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
  }

  protected void updateArchive(List<S> population)
  {
    temporaryArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
    for (S solution : truncatedArchive){
      temporaryArchive.add((S) solution.copy());
    }
    for (S solution : population){
      temporaryArchive.add((S) solution.copy());
      epsilonArchive.add((S) solution.copy());
    }
    List<S> union = temporaryArchive.getSolutionList();
    strengthRawFitness.computeDensityEstimator(union);
    truncatedArchive = environmentalSelection.execute(union);
  }

  @Override protected List<S> replacement(List<S> population, List<S> offspringPopulation) {

    // add offspring to epsilon archive.
    updateArchive(offspringPopulation);

    List<S> jointPopulation = new ArrayList<>();
    jointPopulation.addAll(population);
    jointPopulation.addAll(offspringPopulation);

    RankingAndCrowdingSelection<S> rankingAndCrowdingSelection ;
    rankingAndCrowdingSelection = new RankingAndCrowdingSelection<S>(getMaxPopulationSize(), dominanceComparator) ;

    return rankingAndCrowdingSelection.execute(jointPopulation) ;
  }

  @Override public String getName() {
    return "NSGAIIWithEpsilonArchive" ;
  }

  @Override public String getDescription() {
    return "Nondominated Sorting Genetic Algorithm version II with epsilon archive" ;
  }
}
