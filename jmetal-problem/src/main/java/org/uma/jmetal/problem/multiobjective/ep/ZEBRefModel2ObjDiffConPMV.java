package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
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
 * Variable : setpoint temperature schedule between 5:00 and 24:00 using difference (length:20)
 * Objective 1: power consumption
 * Objective 2: thermal comfort level
 * Constraint 1: exceedance of comfort level
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ZEBRefModel2ObjDiffConPMV extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  /**
   * Constructor.
   */
  public ZEBRefModel2ObjDiffConPMV() {

    setNumberOfVariables(20);
    setNumberOfObjectives(2);
    setNumberOfConstraints(1);
    setName("ZEBRefModel2ObjDiffConPMV") ;

    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;

    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(0.0);
      upperLimit.add(1.0);
    }

    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);

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

    // 快適度に制約がある問題を計算
    EnergyPlusObjectives energyPlusObjectives = new EnergyPlusObjectives(variables);
    double comfortLevel = energyPlusObjectives.calculateAveragePMV();
    double totalElectricEnergy = energyPlusObjectives.calculateTotalElectricEnergy();
    double constraintOfPMV = energyPlusObjectives.countConstraintExceededTimesOfPMV();

    // 評価値と制約違反量を格納
    solution.setObjective(0, Math.abs(comfortLevel));
    solution.setObjective(1, totalElectricEnergy);
    constraintViolation[0] = constraintOfPMV;

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
