package org.uma.jmetal.problem.multiobjective.UF;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.Cast;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem CEC2009_UF12
 */
@SuppressWarnings("serial")
public class UF12 extends AbstractDoubleProblem {

 /**
  * Constructor.
  * Creates a default instance of problem CEC2009_UF12 (30 decision variables)
  */
  public UF12()  {
    this(30);
  }

 /**
  * Creates a new instance of problem CEC2009_UF12.
  * @param numberOfVariables Number of variables.
  */
  public UF12(int numberOfVariables) {
    setNumberOfVariables(numberOfVariables) ;
    setNumberOfObjectives(5) ;
    setNumberOfConstraints(0) ;
    setName("UF12") ;

    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;

    lowerLimit.add(0.0);
    upperLimit.add(1.0);
    for (int i = 1; i < getNumberOfVariables(); i++) {
      lowerLimit.add(-1.0);
      upperLimit.add(1.0);
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    double[] x = new double[getNumberOfVariables()];
    for (int i = 0; i < solution.getNumberOfVariables(); i++) {
      x[i] = solution.getVariableValue(i) ;
    }

    // write variables to csv
    String threadName = Thread.currentThread().getName();
    String baseFolder = "C:\\workspace\\PlatEMO\\package\\UF12Runner\\for_redistribution_files_only\\";
    String exeFileName = baseFolder + "UF12Runner.exe";
    String variableFileName = baseFolder+"var"+threadName+".csv";
    String fitnessFileName = variableFileName.replace(".csv", "") + "_fitness.csv";
    Csv.write(variableFileName, Cast.doubleToString(new Matrix(x, Matrix.DIRECTION_ROW).get()), "", "UTF-8", false);

    // execute UF12Runner.exe
    String command = exeFileName + " " + variableFileName;
    //EnergyPlusプログラム実行
    int ret = 0;
    Runtime runtime = Runtime.getRuntime();
    try{
      Process process = runtime.exec(command);
      ret = process.waitFor();
      System.out.println(ret);
      if(ret!=0)	JMetalLogger.logger.severe("UF12Runner occurred error(s).");
    }catch(Exception e){
      e.printStackTrace();
    }

    // read fitness csv file
    double[][] f = Csv.read(fitnessFileName);

    // set fitness value to solution
    for(int i=0; i<f[0].length; i++){
      solution.setObjective(i, f[0][i]);
    }
  }
}
