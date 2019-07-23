package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.List;

/**
 * Multi-objective Problem of air-conditioning setpoint temperature schedule optimization using EnergyPlus building energy simulator.
 * Building model: reference building model in the ZEB design guideline
 * Variable : setpoint temperature schedule between 6:00 and 24:00 using difference (length:19)
 * Objective 1: thermal comfort level
 * Objective 2: total power consumption
 * Objective 3: dissatisfied level of environment in the case of weather forecast has upward or downward errors
 * Objective 4: increased amount of peak power consumption in the case of weather forecast has upward or downward errors
 * Constraint 1: exceedance of comfort level
 * Constraint 2: peak power consumption
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ZEBRefModel4ObjDiffConPMVPeakPower extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;
  /**
   * Constructor.
   */
  public ZEBRefModel4ObjDiffConPMVPeakPower() {

    setNumberOfVariables(19);
    setNumberOfObjectives(4);
    setNumberOfConstraints(2);
    setName("ZEBRefModel4ObjDiffConPMVPeakPower") ;

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
    maxValue.add(200.0);
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

    // 予報通りの場合を計算
    EnergyPlusObjectives objectivesWithoutError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus.xml");

    // 予報誤差のある場合を計算
    EnergyPlusObjectives objectivesWithUpwardError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus_upward.xml");
    EnergyPlusObjectives objectivesWithDownwardError = new EnergyPlusObjectives(variables, ".\\xml\\energyplus_downward.xml");

    // 目的関数を計算して代入
    double[] fitness = new double[getNumberOfObjectives()];
    fitness[0] = Math.abs( objectivesWithoutError.calculateAveragePMV() );
    fitness[1] = objectivesWithoutError.calculateTotalElectricEnergy();
    double[] peakDissatisfiedRate = new double[3];
    double[] peakPMV = objectivesWithUpwardError.calculatePeakPMV();

    // 制約を計算して代入
    double[] constraints = new double[getNumberOfConstraints()];
    // constraint 1
    constraints[0] = objectivesWithoutError.countConstraintExceededTimesOfPMV();
    // constraint 2
    double[] peakElectricEnergy = new double[3];
    peakElectricEnergy[0] = objectivesWithoutError.calculatePeakElectricEnergy();
    peakElectricEnergy[1] = objectivesWithUpwardError.calculatePeakElectricEnergy();
    peakElectricEnergy[2] = objectivesWithDownwardError.calculatePeakElectricEnergy();
    double peak = new Vector(peakElectricEnergy).max();
    double PEAK_ELECTRIC_ENERGY_MAX = 1e8;
    if(peak > PEAK_ELECTRIC_ENERGY_MAX)
      constraints[1] = peak - PEAK_ELECTRIC_ENERGY_MAX;
    else
      constraints[1] = 0;

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
