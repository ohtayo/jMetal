package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.BuildingUtils;
import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.log.Logging;
import jp.ohtayo.commons.math.Matrix;
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
public class ZEBRefModelLSTMVarDiff2ObjConPMV extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;
  /**
   * Constructor.
   */
  public ZEBRefModelLSTMVarDiff2ObjConPMV() {

    setNumberOfVariables(19);
    setNumberOfObjectives(2);
    setNumberOfConstraints(1);
    setName("ZEBRefModel2ObjDiffConPMV") ;

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
    minValue.add(1.0e9);
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
    // スレッド番号取得
    String threadName = Thread.currentThread().getName();
    String[] splitted = threadName.split("-");
    String threadNumber = splitted[splitted.length-1];
    String workingDirectory = "C:\\workspace\\energyplus_lstm\\"+threadNumber+"\\";

    // 設計変数の変換と書き出し
    List<Double> variablesList = solution.getVariables();
    double[] variables = new double[variablesList.size()];
    for (int i = 0; i < variablesList.size(); i++)  variables[i] = variablesList.get(i);
    double[] temperature = EnergyPlusObjectives.variableToTemperatureSettingUsingDifference(variables, 25, 6,25);
    Csv.write(workingDirectory+"lstm_in.csv", temperature, "settemp");

    // LSTMモデルによるEnergyPlus計算のシミュレーション実行
    JMetalLogger.logger.info("energy plus execution at thread:"+threadName);
    int ret = 0;
    Runtime runtime = Runtime.getRuntime();
    try{
      Process process = runtime.exec(workingDirectory+"evaluator.cmd");
      ret = process.waitFor();
      System.out.println(ret);
      if(ret!=0)	Logging.logger.severe("Python occurred error(s).");
    }catch(Exception e){
      e.printStackTrace();
    }

    // 快適度に制約がある問題を計算
    Matrix result = new Matrix(Csv.read(workingDirectory+"lstm_out.csv", 1, 1));
    double[] fitness = new double[getNumberOfObjectives()];
    double[] constraints = new double[getNumberOfConstraints()];
    fitness[0] = Math.abs( result.getColumn(1).get(6,19).mean() );  // 対象時刻の中間階PMVを取得して平均の絶対値とる
    fitness[1] = result.getColumn(3).sum()*6; // 3列目(消費電力)データを取得して合計値を取る．6回に1回間引かれているので6倍する．
    constraints[0] = result.getColumn(1).get(6,19).abs().round().sum(); //対象時刻の中間階PMVが±0.5をはみ出ている回数を積算する

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
