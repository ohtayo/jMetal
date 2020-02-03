package org.uma.jmetal.util.evaluator.impl;

import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.impl.AbstractAtOneTimeEvaluableDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

/**
 * Evaluator class for at-one-time evaluaton.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class AtOneTimeSolutionListEvaluator<S> implements SolutionListEvaluator<S> {

  @Override
  public List<S> evaluate(List<S> solutionList, Problem<S> problem) throws JMetalException {
    AbstractAtOneTimeEvaluableDoubleProblem prob = (AbstractAtOneTimeEvaluableDoubleProblem)problem;
    JMetalLogger.logger.info("Evaluate solutions at one time.");
    prob.evaluate((List<DoubleSolution>)solutionList);

    return solutionList;
  }

  @Override
  public void shutdown() {
    //This method is an intentionally-blank override.
  }
}
