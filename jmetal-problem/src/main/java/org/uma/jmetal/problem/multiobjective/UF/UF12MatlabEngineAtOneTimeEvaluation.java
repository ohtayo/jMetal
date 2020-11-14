package org.uma.jmetal.problem.multiobjective.UF;

import com.mathworks.engine.EngineException;
import com.mathworks.engine.MatlabEngine;
import org.uma.jmetal.problem.impl.AbstractAtOneTimeEvaluableDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;

/**
 * Class representing problem CEC2009_UF12
 */
@SuppressWarnings("serial")
public class UF12MatlabEngineAtOneTimeEvaluation extends AbstractAtOneTimeEvaluableDoubleProblem {

 /**
  * Constructor.
  * Creates a default instance of problem CEC2009_UF12 (30 decision variables)
  */
  public UF12MatlabEngineAtOneTimeEvaluation()  {
    this(30);
  }

 /**
  * Creates a new instance of problem CEC2009_UF12.
  * @param numberOfVariables Number of variables.
  */
  public UF12MatlabEngineAtOneTimeEvaluation(int numberOfVariables) {
    setNumberOfVariables(numberOfVariables) ;
    setNumberOfObjectives(5) ;
    setNumberOfConstraints(0) ;
    setName("UF12MatlabEngineAtOneTimeEvaluation") ;

    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;

    double[] lowerBound = {-1.773,-1.846,-1.053,-2.37,-1.603,-1.878,-1.677,-0.935,-1.891,-0.964,-0.885,-1.69,-2.235,-1.541,-0.72,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double[] upperBound = {1.403,1.562,2.009,0.976,1.49,1.334,1.074,2.354,1.462,2.372,2.267,1.309,0.842,1.665,2.476,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(lowerBound[i]);
      upperLimit.add(upperBound[i]);
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    return;
  }
  @Override
  public void evaluate(List<DoubleSolution> solutionList) {
    double[][] x = new double[solutionList.size()][getNumberOfVariables()];
    double[][] f = new double[solutionList.size()][getNumberOfObjectives()];

    // get variables from solution
    for (int s = 0; s < solutionList.size(); s++) {
      for (int v = 0; v < getNumberOfVariables(); v++) {
          x[s][v] = solutionList.get(s).getVariableValue(v);
      }
    }

    try {
      // 同期的なMATLAB Engineの取得
      String[] engines = MatlabEngine.findMatlab();
      MatlabEngine matlabEngine = MatlabEngine.connectMatlab(engines[0]);
      // Put the matrix in the MATLAB workspace
      matlabEngine.putVariable("A", x);
      // Evaluate the command on Matlab shared engine
      matlabEngine.eval("B = UF12LocalExe(A);");
      // Get result from the workspace
      double[][] futureVariable = matlabEngine.getVariable("B");
      f = futureVariable;
      matlabEngine.disconnect();
      Thread.sleep(1000);   // wait disconnect()
    } catch (EngineException e) {
      e.printStackTrace();
    } catch (ExecutionException e) {
      e.printStackTrace();
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    // set fitness value to solution
    for (int s = 0; s < solutionList.size(); s++) {
      for (int o = 0; o < getNumberOfObjectives(); o++) {
          solutionList.get(s).setObjective(o, f[s][o]);
      }
    }
  }
}
