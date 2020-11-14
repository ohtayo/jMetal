package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.io.TimeSeries;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import jp.ohtayo.commons.util.Cast;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.sql.Time;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * EnergyPlusを用いLSTMサロゲート評価器と同じ条件(1hourサンプル)で評価する関数
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
@SuppressWarnings("serial")
public class ZEBRefModelVarDiff2ObjConPMVCondLSTM extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;
  /**
   * Constructor.
   */
  public ZEBRefModelVarDiff2ObjConPMVCondLSTM() {

    setNumberOfVariables(19);
    setNumberOfObjectives(2);
    setNumberOfConstraints(1);
    setName("ZEBRefModelVarDiff2ObjConPMVCondLSTM") ;

    // set limit of variables
    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;
    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(0.0);
      upperLimit.add(1.0);
    }
    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);

    // set limit of objectives
    minValue = new ArrayList<>(getNumberOfObjectives());
    maxValue = new ArrayList<>(getNumberOfObjectives());
    // objective1
    minValue.add(0.0);
    maxValue.add(2.0);
    // objective2
    minValue.add(2.0e9);
    maxValue.add(9.0e9);

    constraintViolation = new double[getNumberOfConstraints()];
    for(int i=0; i< getNumberOfConstraints(); i++){
      constraintViolation[i] = 0.0;
    }
    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ; // 制約違反の総量
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ; // 制約違反数
  }

  @Override
  public void evaluate(DoubleSolution solution)  {
    long start = System.currentTimeMillis();
     // スレッド番号取得
    String threadName = Thread.currentThread().getName();
    // 設計変数の変換
    List<Double> variablesList = solution.getVariables();
    double[] variables = new double[variablesList.size()];
    for (int i = 0; i < variablesList.size(); i++)  variables[i] = variablesList.get(i);

    // EnergyPlusのシミュレーション実行
    JMetalLogger.logger.info("energy plus execution at thread:"+threadName);

    // 快適度に制約がある問題を計算
    double[] fitness = new double[getNumberOfObjectives()];
    double[] constraints = new double[getNumberOfConstraints()];
    EnergyPlusObjectives energyPlusObjectives = new EnergyPlusObjectives(variables);
    TimeSeries allData = new TimeSeries( energyPlusObjectives.get() );
    DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
    String dateTime = dateTimeFormatter.format(LocalDateTime.now());
    String header = "time, outdoortemp, outdoorhumi, settemp, groundtemp, groundhumi, middletem, middlehumi, toptemp, tophumi, groundpmv, middlepmv, toppmv, electricenergy, coolingenergy";
    allData.write("eplusout_"+dateTime+".csv", header);

    Vector temp = new Vector(5,6, allData.length()-1);
    Vector index = temp.add(new Vector(1,0));
    index.sort();
    Vector thinPmv = allData.getColumn(11).get(Cast.doubleToInt(index.get()));
    Vector targetPmv = thinPmv.get(7, 15);
    Vector thinEnergy = allData.getColumn(13).get(Cast.doubleToInt(index.get()));
    fitness[0] = Math.abs( targetPmv.mean() );
//    fitness[1] = allData.getColumn(13).sum();
    fitness[1] = thinEnergy.sum() * 6;
    constraints[0] = targetPmv.abs().round().sum(); // 0.5を超過した

    // Normalize objective values
    double[] normalizedFitness = new double[getNumberOfObjectives()];
    for(int o=0; o<getNumberOfObjectives(); o++) {
      normalizedFitness[o] = (fitness[o] - minValue.get(o)) / (maxValue.get(o) - minValue.get(o));
    }

    // 評価値と制約違反量を格納
    for(int o=0; o<getNumberOfObjectives(); o++) {
      solution.setObjective(o, normalizedFitness[o]);
    }
    constraintViolation = constraints;

    this.evaluateConstraints(solution);
    System.out.println((System.currentTimeMillis()-start)+"ms");
  }

  /** EvaluateConstraints() method */
  private void evaluateConstraints(DoubleSolution solution)  {

    double overallConstraintViolation = 0.0;
    int violatedConstraints = 0;
    for (int i = 0; i < getNumberOfConstraints(); i++) {
      if ( (-1 * constraintViolation[i]) <0.0){
        overallConstraintViolation-=constraintViolation[i]; //制約違反量に負の値を与える
        violatedConstraints++;
      }
    }

    overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolation);
    numberOfViolatedConstraints.setAttribute(solution, violatedConstraints);
  }
}
