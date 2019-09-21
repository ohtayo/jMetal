package org.uma.jmetal.runner.multiobjective;

import java.io.FileNotFoundException;
import java.util.List;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaiii.NSGAIII;
import org.uma.jmetal.algorithm.multiobjective.nsgaiii.NSGAIIIBuilder;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.*;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;

import org.apache.commons.io.FilenameUtils;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.StringUtility;

/**
 * Class to configure and run the NSGA-III algorithm (parallel version)
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
public class ParallelNSGAIIIRunner extends AbstractAlgorithmRunner {
  /**
   * @param args Command line arguments.
   * @throws java.io.IOException
   * @throws SecurityException
   * @throws ClassNotFoundException
   */
  public static void main(String[] args) throws JMetalException, FileNotFoundException {
    // 変数定義
    Problem<DoubleSolution> problem;
    Algorithm<List<DoubleSolution>> algorithm;
    CrossoverOperator<DoubleSolution> crossover;
    MutationOperator<DoubleSolution> mutation;
    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection;

    // 引数処理：目的関数，参照パレートフロント，スレッド数の決定
    String problemName;
    int numberOfIndividuals = 100;
    int numberOfGenerations = 100;
    int numberOfThreads = 1;
    String referenceParetoFront = "" ;
    String fileNameOfInitialSolutions="";
    if (args.length == 1) {
      problemName = args[0];
    } else if (args.length == 2) {
      problemName = args[0];
      numberOfIndividuals = Integer.valueOf(args[1]);
    } else if (args.length == 3){
      problemName = args[0];
      numberOfIndividuals = Integer.valueOf(args[1]);
      numberOfGenerations = Integer.valueOf(args[2]);
    } else if (args.length == 4){
      problemName = args[0];
      numberOfIndividuals = Integer.valueOf(args[1]);
      numberOfGenerations = Integer.valueOf(args[2]);
      numberOfThreads = Integer.valueOf(args[3]);
    } else if (args.length == 5){
      problemName = args[0];
      numberOfIndividuals = Integer.valueOf(args[1]);
      numberOfGenerations = Integer.valueOf(args[2]);
      numberOfThreads = Integer.valueOf(args[3]);
      referenceParetoFront = args[4];
    } else if (args.length == 6){
      problemName = args[0];
      numberOfIndividuals = Integer.valueOf(args[1]);
      numberOfGenerations = Integer.valueOf(args[2]);
      numberOfThreads = Integer.valueOf(args[3]);
      referenceParetoFront = args[4];
      fileNameOfInitialSolutions = args[5];
    } else {
//      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModel2ObjDiffConPMV";
      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";
//      problemName = "org.uma.jmetal.problem.multiobjective.zdt.ZDT1";
      numberOfThreads = 6;
      numberOfGenerations = 10;
      numberOfIndividuals = 10;
      fileNameOfInitialSolutions = "";
      //fileNameOfInitialSolutions = "C:\\workspace\\jMetal\\initialSolutions.csv";
    }

    // 目的関数の設定
    problem = ProblemUtils.loadProblem(problemName);

    // 各種パラメータ，オペレータの指定
    double crossoverProbability = 0.9 ;
    double crossoverDistributionIndex = 30.0 ;
    crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex) ;

    double mutationProbability = 1.0 / problem.getNumberOfVariables() ;
    double mutationDistributionIndex = 20.0 ;
    mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex) ;
    
    selection = new BinaryTournamentSelection<DoubleSolution>();

    // マルチスレッドの評価
    SolutionListEvaluator<DoubleSolution> evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>( numberOfThreads, problem ) ;

    // NSGAIIIのパラメータ定義
    NSGAIIIBuilder<DoubleSolution> builder = new NSGAIIIBuilder<>(problem)
      .setCrossoverOperator(crossover)
      .setMutationOperator(mutation)
      .setSelectionOperator(selection)
      .setMaxIterations(numberOfGenerations)
      .setPopulationSize(numberOfIndividuals)
      .setSolutionListEvaluator(evaluator) ;
    algorithm = builder.build() ;

    // 初期値のファイル名の指定があれば初期値を設定
    if(!StringUtility.isNullOrEmpty(fileNameOfInitialSolutions)) {
      List<DoubleSolution> initialPopulation = ((NSGAIII<DoubleSolution>) algorithm).createInitialPopulation();
      Matrix data = new Matrix(Csv.read(fileNameOfInitialSolutions));
      for (int r = 0; r < initialPopulation.size(); r++) {
          for (int c = 0; c < data.columnLength(); c++) {
              initialPopulation.get(r).setVariableValue(c, data.get(r, c));
          }
      }
      ((NSGAIII<DoubleSolution>) algorithm).setInitialPopulation(initialPopulation);
      JMetalLogger.logger.info("Use initial population: "+ FilenameUtils.getName(fileNameOfInitialSolutions));
    }

    // アルゴリズム実行
    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
        .execute() ;
    builder.getSolutionListEvaluator().shutdown();

    // 探索結果と計算結果の取得
    List<DoubleSolution> population = algorithm.getResult() ;
    long computingTime = algorithmRunner.getComputingTime() ;
    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

    if (!referenceParetoFront.equals("")) {
      printQualityIndicators(population, referenceParetoFront) ;
    }

    // 評価スレッドの終了処理
    evaluator.shutdown();

    // 最終解の出力
    new SolutionListOutput(population)
            .setSeparator("\t")
            .setVarFileOutputContext(new DefaultFileOutputContext("VAR.tsv"))
            .setFunFileOutputContext(new DefaultFileOutputContext("FUN.tsv"))
            .print() ;
    JMetalLogger.logger.info("Objectives values have been written to file FUN.tsv");
    JMetalLogger.logger.info("Variables values have been written to file VAR.tsv");
  }
}
