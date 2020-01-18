package org.uma.jmetal.problem.multiobjective.UF;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.Cast;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

/**
 * Class representing problem CEC2009_UF12
 */
@SuppressWarnings("serial")
public class UF12 extends AbstractDoubleProblem {
  public double[] Lambda;
  public double[][] M;
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

    double[] lowerBound = {-1.773,-1.846,-1.053,-2.37,-1.603,-1.878,-1.677,-0.935,-1.891,-0.964,-0.885,-1.69,-2.235,-1.541,-0.72,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    double[] upperBound = {1.403,1.562,2.009,0.976,1.49,1.334,1.074,2.354,1.462,2.372,2.267,1.309,0.842,1.665,2.476,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(lowerBound[i]);
      upperLimit.add(upperBound[i]);
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);

    // init parameter lambda
    double[] temp = {0.113,0.105,0.117,0.119,0.108,0.11,0.101,0.107,0.111,0.109,0.12,0.108,0.101,0.105,0.116,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
    Lambda = new double[temp.length];
    for(int i=0; i<temp.length; i++){
        Lambda[i] = temp[i];
    }

    // init rotation matrix
    String dataDirectory = "MOEAD_Weights";
    String dataFileName = "UF12M.csv";
    URL url = getClass().getClassLoader().getResource(dataDirectory + "/" + dataFileName);
                    //.getResourceAsStream(dataDirectory + "/" + dataFileName);
    M = Csv.read(url.getFile());
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    double[] x = new double[getNumberOfVariables()];
    double[] f = new double[getNumberOfVariables()];
    for (int i = 0; i < solution.getNumberOfVariables(); i++) {
      x[i] = solution.getVariableValue(i) ;
    }
    Matrix PopDec = new Matrix(x, Matrix.DIRECTION_ROW);
    Matrix lambda = new Matrix(Lambda, Matrix.DIRECTION_ROW);
    Matrix m = new Matrix(M);
    Matrix z     = PopDec.multiply(m.transpose());
    Matrix p     = new Matrix(z.length(), z.columnLength(), 0.0);
    int r=0;
    for(int c=0; c<p.columnLength(); c++){
      if(z.get(r,c)<0){
        p.set(r, c, -1.0*z.get(r, c));
        z.set(r, c, -1.0*lambda.get(r,c)*z.get(r,c));
      }
      else if(z.get(r,c)>1){
        p.set(r, c, z.get(r, c)-1.0);
        z.set(r, c, 1.0-lambda.get(r,c)*(z.get(r,c)-1.0));
      }
    }
    Matrix psum = new Matrix(getNumberOfVariables(), getNumberOfObjectives(), 0.0);
    for(int i=0; i<psum.columnLength(); i++){

    }
    // Todo: implement following equations
/*
    for i = 1 : 5
      psum(:,i) = sqrt(sum(p(:,[1:min(6-i,4),5:end]).^2,2));
    end
            g      = 100*(26+sum((z(:,5:end)-0.5).^2-cos(20.*pi.*(z(:,5:end)-0.5)),2));
    PopObj = 2./(1+exp(-psum)).*(1+repmat(1+g,1,5).*fliplr(cumprod([ones(size(g,1),1),cos(z(:,1:5-1)*pi/2)],2)).*[ones(size(g,1),1),sin(z(:,5-1:-1:1)*pi/2)]);
*/

    // set fitness value to solution
    for(int i=0; i<f.length; i++){
      solution.setObjective(i, f[i]);
    }
  }
}
