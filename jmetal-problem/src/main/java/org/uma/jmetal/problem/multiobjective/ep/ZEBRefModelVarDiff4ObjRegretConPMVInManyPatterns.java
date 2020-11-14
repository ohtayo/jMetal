package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.io.TimeSeries;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

/**
 * Multi-objective Problem of air-conditioning setpoint temperature schedule optimization using EnergyPlus building energy simulator.
 * Building model: reference building model in the ZEB design guideline
 * Variable : setpoint temperature schedule between 6:00 and 24:00 using difference (length:19)
 * Objective 1: thermal comfort level
 * Objective 2: total power consumption
 * Objective 3: amount of deterioration of PMV in the case of weather forecast has upward or downward errors
 * Objective 4: amount of deterioration of total power consumption in the case of weather forecast has upward or downward errors
 * Constraint 1: exceedance of comfort level
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
@SuppressWarnings("serial")
public class ZEBRefModelVarDiff4ObjRegretConPMVInManyPatterns extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;
  /**
   * Constructor.
   */
  public ZEBRefModelVarDiff4ObjRegretConPMVInManyPatterns() {

    setNumberOfVariables(19);
    setNumberOfObjectives(4);
    setNumberOfConstraints(1);
    setName("ZEBRefModelVarDiff4ObjRegretConPMVInManyPatterns") ;

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
    // objective3
    minValue.add(0.0);
    maxValue.add(2.0);
    // objective4
    minValue.add(0.0);
    maxValue.add(7.0e9);

    constraintViolation = new double[getNumberOfConstraints()];
    for(int i=0; i< getNumberOfConstraints(); i++){
      constraintViolation[i] = 0.0;
    }
    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ; // 制約違反の総量
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ; // 制約違反数
  }

  @Override
  public void evaluate(DoubleSolution solution)  {
    // スレッド番号取得
    String threadName = Thread.currentThread().getName();
    // 設計変数の変換
    List<Double> variablesList = solution.getVariables();
    double[] variables = new double[variablesList.size()];
    for (int i = 0; i < variablesList.size(); i++)  variables[i] = variablesList.get(i);

    // EnergyPlusのシミュレーション実行
    JMetalLogger.logger.info("energy plus execution at thread:"+threadName);
/*
    // 予報通りの場合を計算
    EnergyPlusObjectives objectivesWithoutError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus.xml");

    // 予報誤差のある場合を計算
    EnergyPlusObjectives objectivesWithUpwardError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus_upward.xml");
    EnergyPlusObjectives objectivesWithDownwardError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus_downward.xml");

    // 時系列データの保存
    saveTimeSeriesData(new TimeSeries( objectivesWithoutError.get() ), "WithoutError");
    saveTimeSeriesData(new TimeSeries( objectivesWithUpwardError.get() ), "UpwardError");
    saveTimeSeriesData(new TimeSeries( objectivesWithDownwardError.get() ), "DownwardError");
*/
    // さらに多くの予報誤差のある場合を計算
    EnergyPlusObjectives[] objectivesWithError = new EnergyPlusObjectives[10];
    double[][] p = new double[2][10];
    for(int i=1; i<=10; i++) {
      String suffix;
      if(i<=5)  suffix = "up" + String.valueOf(i);
      else      suffix = "down" + String.valueOf(i-5);
      objectivesWithError[i-1] = new EnergyPlusObjectives(variables).setXmlFile(".\\xml\\energyplus_"+suffix+".xml").calculate();
      saveTimeSeriesData(new TimeSeries( objectivesWithError[i-1].get() ), suffix);
      p[0][i-1] = objectivesWithError[i-1].calculateAveragePMV();
      p[1][i-1] = objectivesWithError[i-1].calculateTotalElectricEnergy();
    }
    // 予報誤差がある場合のPMVと消費電力値も保存しておく．
    DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
    String dateTime = dateTimeFormatter.format(LocalDateTime.now());
    Csv.write("PMVAndPower_"+dateTime+".csv", p);

    // 目的関数を計算して代入
    double[] fitness = new double[getNumberOfObjectives()];
    double[] pmv = new double[3];
    double[] power = new double[3];
    pmv[0] = Math.abs( objectivesWithError[0].calculateAveragePMV() );
    pmv[1] = Math.abs( objectivesWithError[1].calculateAveragePMV() );
    pmv[2] = Math.abs( objectivesWithError[2].calculateAveragePMV() );
    power[0] = objectivesWithError[0].calculateTotalElectricEnergy();
    power[1] = objectivesWithError[1].calculateTotalElectricEnergy();
    power[2] = objectivesWithError[2].calculateTotalElectricEnergy();
    fitness[0] = pmv[0];
    fitness[1] = power[0];
    fitness[2] = Math.max( Math.abs(pmv[1]-pmv[0]), Math.abs(pmv[2]-pmv[0]) );
    fitness[3] = Math.max( Math.abs(power[1]-power[0]), Math.abs(power[2]-power[0]) );

    // Normalize and set objective values
    double[] normalizedFitness = new double[getNumberOfObjectives()];
    for(int o=0; o<getNumberOfObjectives(); o++) {
      normalizedFitness[o] = (fitness[o] - minValue.get(o)) / (maxValue.get(o) - minValue.get(o));
    }
    for(int o=0; o<getNumberOfObjectives(); o++) {
      solution.setObjective(o, normalizedFitness[o]);
    }

    // set constraints
    double[] constraints = new double[getNumberOfConstraints()];
    constraints[0] = objectivesWithError[0].countConstraintExceededTimesOfPMV();

    constraintViolation = constraints;

    this.evaluateConstraints(solution);
  }

  // シミュレータの計算した時系列データを保存
  private void saveTimeSeriesData(TimeSeries data, String suffix){
    DateTimeFormatter dateTimeFormatter = DateTimeFormatter.ofPattern("yyyyMMdd_HHmmss_SSS");
    String dateTime = dateTimeFormatter.format(LocalDateTime.now());
    String header = "time, outdoortemp, outdoorhumi, settemp, groundtemp, groundhumi, middletem, middlehumi, toptemp, tophumi, groundpmv, middlepmv, toppmv, electricenergy, coolingenergy";
    data.write("eplusout_"+dateTime+"_"+suffix+".csv", header);
  }

  /** EvaluateConstraints() method */
  private void evaluateConstraints(DoubleSolution solution)
  {
    double overallConstraintViolation = 0.0;
    int violatedConstraints = 0;
    for (int i = 0; i < getNumberOfConstraints(); i++) {
      if ( (-1 * constraintViolation[i]) <0.0){
        overallConstraintViolation -= constraintViolation[i]; //制約違反量に負の値を与える
        violatedConstraints++;
      }
    }

    overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolation);
    numberOfViolatedConstraints.setAttribute(solution, violatedConstraints);
  }
}
