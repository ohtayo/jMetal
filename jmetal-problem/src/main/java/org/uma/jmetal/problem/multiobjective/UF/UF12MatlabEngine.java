package org.uma.jmetal.problem.multiobjective.UF;

import com.mathworks.engine.EngineException;
import com.mathworks.engine.MatlabEngine;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutionException;

/**
 * Class representing problem CEC2009_UF12 using matlab engine
 */
@SuppressWarnings("serial")
public class UF12MatlabEngine extends AbstractDoubleProblem {

 /**
  * Constructor.
  * Creates a default instance of problem CEC2009_UF12 (30 decision variables)
  */
  public UF12MatlabEngine()  {
    this(30);
  }

 /**
  * Creates a new instance of problem CEC2009_UF12.
  * @param numberOfVariables Number of variables.
  */
  public UF12MatlabEngine(int numberOfVariables) {
    setNumberOfVariables(numberOfVariables) ;
    setNumberOfObjectives(5) ;
    setNumberOfConstraints(0) ;
    setName("UF12_Engine") ;

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
    double[][] x = new double[1][getNumberOfVariables()];
    double[] f = new double[getNumberOfObjectives()];

    // get variables from solution
    for (int i = 0; i < solution.getNumberOfVariables(); i++) {
      x[0][i] = solution.getVariableValue(i) ;
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
        double[] futureVariable = matlabEngine.getVariable("B");
        f = futureVariable;
        matlabEngine.disconnect();
        Thread.sleep(1000);   // wait disconnect()
    }catch (EngineException e){
        e.printStackTrace();
    }catch(ExecutionException e){
        e.printStackTrace();
    }catch (InterruptedException e){
        e.printStackTrace();
    }

    // set fitness value to solution
    for(int i=0; i<f.length; i++){
      solution.setObjective(i, f[i]);
    }
  }
}
