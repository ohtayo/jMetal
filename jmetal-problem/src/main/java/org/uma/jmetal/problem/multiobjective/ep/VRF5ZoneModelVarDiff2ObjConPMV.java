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
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
@SuppressWarnings("serial")
public class VRF5ZoneModelVarDiff2ObjConPMV extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;
  /**
   * Constructor.
   */
  public VRF5ZoneModelVarDiff2ObjConPMV() {

    setNumberOfVariables(19);
    setNumberOfObjectives(2);
    setNumberOfConstraints(1);
    setName("VRF5ZoneModelVarDiff2ObjConPMV") ;

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
    minValue.add(2.0e8);
    maxValue.add(9.0e8);

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
    EnergyPlusObjectives energyPlusObjectives
            = new EnergyPlusObjectives(variables)
            .setXmlFile("./xml/energyplus_vrf5z.xml")
            .setIdfOffsets(242 -1, 521 -1)
            .calculate();
    fitness[0] = Math.abs( energyPlusObjectives.calculateAveragePMV() );
    fitness[1] = energyPlusObjectives.calculateTotalElectricEnergy();
    constraints[0] = energyPlusObjectives.countConstraintExceededTimesOfPMV();

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

    // for debug 制約がAttributeに反映されない場合があるので，反映されていなければ無理やり値を入れる
    // Todo しきい値は要検討
    OverallConstraintViolation<DoubleSolution> overallConstraintViolation;
    overallConstraintViolation = new OverallConstraintViolation<DoubleSolution>() ;
    double violation = overallConstraintViolation.getAttribute(solution);
    double comfort = solution.getObjectives()[0]*2;
    double power = solution.getObjectives()[1];
    if( (violation==0) && ((comfort>0.5)||(power<0.1)) ){
      JMetalLogger.logger.severe("illegal objective values.");
      for(int o=0; o<getNumberOfObjectives(); o++) {
        solution.setObjective(o, maxValue.get(o));
      }
      overallConstraintViolationDegree.setAttribute(solution, 100.0);
      numberOfViolatedConstraints.setAttribute(solution, 1);

    }

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
