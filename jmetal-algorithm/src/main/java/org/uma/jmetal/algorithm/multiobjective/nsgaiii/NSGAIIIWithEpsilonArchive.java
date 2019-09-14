package org.uma.jmetal.algorithm.multiobjective.nsgaiii;

import org.uma.jmetal.algorithm.impl.AbstractGeneticAlgorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaiii.util.EnvironmentalSelection;
import org.uma.jmetal.algorithm.multiobjective.nsgaiii.util.ReferencePoint;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.solutionattribute.Ranking;
import org.uma.jmetal.util.solutionattribute.impl.DominanceRanking;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

/**
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class NSGAIIIWithEpsilonArchive<S extends Solution<?>> extends NSGAIII<S> {
  private NonDominatedSolutionListArchive<S> epsilonArchive;
  private double eta = 0.0075;

  /** Constructor */
  public NSGAIIIWithEpsilonArchive(NSGAIIIBuilder<S> builder) { // can be created from the NSGAIIIBuilder within the same package
    super(builder) ;
    epsilonArchive = new NonDominatedSolutionListArchive<S>(new DominanceComparator<S>(eta));
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
    dump();
  }

  @Override
  protected void updateProgress() {
    iterations++ ;
    dump();
  }
  protected void updateArchive(List<S> population)
  {
    for (S solution : population){
      epsilonArchive.add((S) solution.copy());
    }
  }

  protected void dump(){
    // dump solution list in the searching
    List<S> result = getResult();
    new SolutionListOutput(result)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/variable" + iterations + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/fitness" + iterations + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<S>(result)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/constraint" + iterations + ".csv"))
            .setSeparator(",")
            .print();
    List<S> allPopulation = getPopulation();
    new SolutionListOutput(allPopulation)
            .setVarFileOutputContext(new DefaultFileOutputContext("./result/variableall" + iterations + ".csv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("./result/fitnessall" + iterations + ".csv"))
            .setSeparator(",")
            .print();
    new ConstraintListOutput<S>(result)
            .setConFileOutputContext(new DefaultFileOutputContext("./result/constraintall" + iterations + ".csv"))
            .setSeparator(",")
            .print();
    List<S> epsilon = epsilonArchive.getSolutionList();
    new SolutionListOutput(epsilon)
        .setVarFileOutputContext(new DefaultFileOutputContext("./result/epsilonVariable" + iterations + ".csv"))
        .setFunFileOutputContext(new DefaultFileOutputContext("./result/epsilonFitness" + iterations + ".csv"))
        .setSeparator(",")
        .print();
    new ConstraintListOutput<S>(epsilon)
        .setConFileOutputContext(new DefaultFileOutputContext("./result/epsilonConstraint" + iterations + ".csv"))
        .setSeparator(",")
        .print();
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
