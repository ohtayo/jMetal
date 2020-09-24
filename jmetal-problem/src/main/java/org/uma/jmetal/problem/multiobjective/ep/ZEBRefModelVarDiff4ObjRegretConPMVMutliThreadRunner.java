package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadPoolExecutor;

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
public class ZEBRefModelVarDiff4ObjRegretConPMVMutliThreadRunner {
  /**
   * @param args Command line arguments.
   * @throws JMetalException
   * Invoking command:
  java org.uma.jmetal.runner.multiobjective.ParallelOMOPSORunner problemName [referenceFront]
   */
  public static void main(String[] args) throws JMetalException {

    int numberOfThread;
    String variablesFileName;
    String objectivesFileName;

    // 引数の処理
    if (args.length == 3) {
      numberOfThread = Integer.valueOf(args[0]);
      variablesFileName = args[1];
      objectivesFileName = args[2];
    } else {
      throw new JMetalException("Illegal argument(s).");
    }

    // スレッドを作って評価する．
    DoubleProblem problem = new ZEBRefModelVarDiff4ObjRegretConPMV();
    List<DoubleSolution> solutionList = new ArrayList<>();
    for(int t=0; t<numberOfThread; t++) solutionList.add(problem.createSolution());
    readVariables(variablesFileName, solutionList);
    ThreadPoolSolutionListEvaluator threadPoolSolutionListEvaluator = new ThreadPoolSolutionListEvaluator(numberOfThread, problem);
    threadPoolSolutionListEvaluator.evaluate(solutionList, problem);
    writeObjectives(objectivesFileName, solutionList);
  }

  /**
   * テキストファイルから設計変数を読み取る
   * @param variablesFileName 設計変数ファイル名
   * @param solutionList 解集合
   */
  private static void readVariables(String variablesFileName, List<DoubleSolution> solutionList){
    try{
      Path path = Paths.get(variablesFileName);
      List<String> lines = Files.readAllLines(path);
      for(int line=0; line<lines.size(); line++) {
        String[] variables = lines.get(line).split(",");
        for (int i = 0; i < solutionList.size(); i++){
          solutionList.get(i).setVariableValue(i, Double.valueOf(variables[i]));
        }
      }
    }catch(IOException e){
      e.printStackTrace();
    }
  }

  /**
   * テキストファイルに目的関数を書き込む
   * @param objectivesFileName 目的関数ファイル名
   * @param solutionList 解集合
   */
  private static void writeObjectives(String objectivesFileName, List<DoubleSolution> solutionList){
    OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ; // 制約違反の総量
    NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ; // 制約違反数
    double degree;
    double constraints;
    try{
      Path path = Paths.get(objectivesFileName);
      if (!Files.exists(path)) Files.createFile(path);  // ファイルがなければ作成
      String objectivesString = "";
      List<String> lines = new ArrayList<>();
      for(int s=0; s<solutionList.size(); s++) {
        for (Double objectives : solutionList.get(s).getObjectives()) objectivesString += String.format("%.3f,", objectives);
        degree = overallConstraintViolationDegree.getAttribute(solutionList.get(s));
        constraints = numberOfViolatedConstraints.getAttribute(solutionList.get(s));
        objectivesString += String.format("%.3f,",degree);
        objectivesString += String.format("%d",constraints);
        lines.add(objectivesString);
      }
      Files.write(path, lines, StandardOpenOption.WRITE);
    }catch(IOException e){
      e.printStackTrace();
    }
  }

}
