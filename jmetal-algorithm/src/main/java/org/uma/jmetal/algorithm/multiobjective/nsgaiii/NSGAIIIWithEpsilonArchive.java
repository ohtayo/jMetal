package org.uma.jmetal.algorithm.multiobjective.nsgaiii;

import org.uma.jmetal.algorithm.multiobjective.nsgaiii.util.EnvironmentalSelection;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.solutionattribute.Ranking;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.util.ArrayList;
import java.util.List;

/**
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class NSGAIIIWithEpsilonArchive<S extends Solution<?>> extends NSGAIII<S> {
  private double eta = 0.0075;
  private NonDominatedSolutionListArchive<S> epsilonArchive;  // epsilon archive
  private NonDominatedSolutionListArchive<S> temporaryArchive;  // 一時アーカイブ
  private List<S> truncatedArchive; // SPEA2の端切りアーカイブ
  private StrengthRawFitness<S> strengthRawFitness; // SPEA2の適合度
  private org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection<S> environmentalSelection; // SPEA2の環境選択

  /** Constructor */
  public NSGAIIIWithEpsilonArchive(NSGAIIIBuilder<S> builder) { // can be created from the NSGAIIIBuilder within the same package
    super(builder);
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
    truncatedArchive = new ArrayList<>(builder.getArchiveSize());
    strengthRawFitness = new StrengthRawFitness<S>();
    environmentalSelection = new org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection<S>(builder.getArchiveSize());
  }

  public void setEta(double eta) {
    this.eta = eta ;
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
  }

  public double getEta(){
    return eta;
  }

  @Override
  protected void initProgress() {
    iterations = 1 ;
    updateArchive(population);
    dump(getPopulation());
    dump(getResult(), "nonDominated");
    dump(epsilonArchive.getSolutionList(), "epsilon");
    dump(truncatedArchive, "truncated");
  }

  @Override
  protected void updateProgress() {
    iterations++ ;
    dump(getPopulation());
    dump(getResult(), "nonDominated");
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

  @Override
  protected List<S> replacement(List<S> population, List<S> offspringPopulation) {

    // add offspring to epsilon archive
    updateArchive(offspringPopulation);

    List<S> jointPopulation = new ArrayList<>();
    jointPopulation.addAll(population) ;
    jointPopulation.addAll(offspringPopulation) ;

    Ranking<S> ranking = computeRanking(jointPopulation);

    //List<Solution> pop = crowdingDistanceSelection(ranking);
    List<S> pop = new ArrayList<>();
    List<List<S>> fronts = new ArrayList<>();
    int rankingIndex = 0;
    int candidateSolutions = 0;
    while (candidateSolutions < getMaxPopulationSize()) {
      fronts.add(ranking.getSubfront(rankingIndex));
      candidateSolutions += ranking.getSubfront(rankingIndex).size();
      if ((pop.size() + ranking.getSubfront(rankingIndex).size()) <= getMaxPopulationSize())
        addRankedSolutionsToPopulation(ranking, rankingIndex, pop);
      rankingIndex++;
    }

    // A copy of the reference list should be used as parameter of the environmental selection
    EnvironmentalSelection<S> selection =
        new EnvironmentalSelection<>(fronts,getMaxPopulationSize(),getReferencePointsCopy(),
            getProblem().getNumberOfObjectives());

    pop = selection.execute(pop);

    return pop;
  }

  @Override public String getName() {
    return "NSGAIIIWithEpsilonArchive" ;
  }

  @Override public String getDescription() {
    return "Non-dominated Sorting Genetic Algorithm version III with epsilon archive" ;
  }

}
