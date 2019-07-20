package org.uma.jmetal.util.archive.impl;

import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.comparator.DominanceComparator;

import java.util.Collections;
import java.util.Comparator;

/**
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class BoundedDominanceArchive<S extends Solution<?>> extends AbstractBoundedArchive<S> {
  private Comparator<S> dominanceComparator;

  public BoundedDominanceArchive(int maxSize) {
    super(maxSize);
    dominanceComparator = new DominanceComparator<S>() ;
  }

  @Override
  public void prune() {
    if (getSolutionList().size() > getMaxSize()) {
      S worst = new SolutionListUtils().findWorstSolution(getSolutionList(), dominanceComparator) ;
      getSolutionList().remove(worst);
    }
  }

  @Override
  public Comparator<S> getComparator() {
    return dominanceComparator ;
  }
  @Override
  public void computeDensityEstimator() {
    // do nothing
  }

  @Override
  public void sortByDensityEstimator() {
    Collections.sort(getSolutionList(), new DominanceComparator<S>());
  }
}
