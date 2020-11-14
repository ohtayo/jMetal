package org.uma.jmetal.util.archive.impl;

import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.comparator.CrowdingDistanceComparator;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.solutionattribute.DensityEstimator;
import org.uma.jmetal.util.solutionattribute.impl.CrowdingDistance;

import java.util.Collections;
import java.util.Comparator;

/**
 * Created by Yoshihiro Ohta on 04/06/2020.
 */
@SuppressWarnings("serial")
public class EpsilonCrowdingDistanceArchive<S extends Solution<?>> extends NonDominatedSolutionListArchive<S> {

  private int maxSize;
  private CrowdingDistanceComparator<S> crowdingDistanceComparator;
  private DensityEstimator<S> crowdingDistance ;

  public EpsilonCrowdingDistanceArchive(int maxSize, double eta) {
    super(new DominanceComparator<>(eta));
    this.maxSize = maxSize;
    crowdingDistanceComparator = new CrowdingDistanceComparator<S>();
    crowdingDistance = new CrowdingDistance<S>() ;
  }

  @Override
  public boolean add(S solution) {
    boolean success = super.add(solution);
    if (success) {
      prune();
    }

    return success;
  }

  public void prune() {
    if (size() > maxSize) {
      crowdingDistance.computeDensityEstimator(getSolutionList());
      S worst = new SolutionListUtils().findWorstSolution(getSolutionList(), crowdingDistanceComparator) ;
      getSolutionList().remove(worst);
    }
  }
}
