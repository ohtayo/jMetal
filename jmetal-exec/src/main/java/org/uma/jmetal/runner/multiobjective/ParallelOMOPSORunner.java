package org.uma.jmetal.runner.multiobjective;

import java.util.List;

import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.omopso.OMOPSO;
import org.uma.jmetal.algorithm.multiobjective.omopso.OMOPSOBuilder;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;

import org.apache.commons.io.FilenameUtils;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.StringUtility;

/**
 * Class for configuring and running the OMOPSO algorithm (parallel version)
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */

public class ParallelOMOPSORunner extends AbstractAlgorithmRunner {
  /**
   * @param args Command line arguments.
   * @throws SecurityException
   * Invoking command:
  java org.uma.jmetal.runner.multiobjective.ParallelOMOPSORunner problemName [referenceFront]
   */
  public static void main(String[] args) throws JMetalException {
    // 変数定義
    DoubleProblem problem;
    Algorithm<List<DoubleSolution>> algorithm;

    // 引数処理：目的関数，参照パレートフロント，スレッド数の決定
    String problemName;
    int numberOfThreads = 1;
    int iterations = 100;
    int particles = 100;
    String fileNameOfInitialSolutions = "";
    if (args.length == 1) {
      problemName = args[0];
    } else if (args.length == 2) {
      problemName = args[0];
      numberOfThreads = Integer.valueOf(args[1]);
    } else if (args.length == 3){
      problemName = args[0];
      numberOfThreads = Integer.valueOf(args[1]);
      iterations = Integer.valueOf(args[2]);
    } else if (args.length == 4){
      problemName = args[0];
      numberOfThreads = Integer.valueOf(args[1]);
      iterations = Integer.valueOf(args[2]);
      particles = Integer.valueOf(args[3]);
    } else if (args.length == 5){
      problemName = args[0];
      numberOfThreads = Integer.valueOf(args[1]);
      iterations = Integer.valueOf(args[2]);
      particles = Integer.valueOf(args[3]);
      fileNameOfInitialSolutions = args[4];
    } else {
//      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModel2ObjDiffConPMV";
      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff2ObjConPMV";
//      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModel4ObjDiffConPMV";
//      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";
//      problemName = "org.uma.jmetal.problem.multiobjective.zdt.ZDT1";
      numberOfThreads = 6;
      iterations = 500;
      particles = 30;
      fileNameOfInitialSolutions = "";
      //fileNameOfInitialSolutions = "C:\\workspace\\jMetal\\initialSolutions.csv";
    }

    // 目的関数の定義
    problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);

    // 突然変異確率
    double mutationProbability = 1.0 / problem.getNumberOfVariables() ;

    // マルチスレッドの評価クラスの設定
    SolutionListEvaluator<DoubleSolution> evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>( numberOfThreads, problem ) ;

    // OMOPSOのパラメータ定義
    OMOPSOBuilder builder = new OMOPSOBuilder(problem, evaluator)
            .setMaxIterations(iterations)
            .setSwarmSize(particles)
            .setUniformMutation(new UniformMutation(mutationProbability, 0.5))
            .setNonUniformMutation(new NonUniformMutation(mutationProbability, 0.5, iterations));
    algorithm = builder.build();

    // 初期値のファイル名の指定があれば初期値を設定
    if(!StringUtility.isNullOrEmpty(fileNameOfInitialSolutions)){
      List<DoubleSolution> initialSwarm = ((OMOPSO) algorithm).createInitialSwarm();
      Matrix data = new Matrix(Csv.read(fileNameOfInitialSolutions));
      for (int r=0; r<initialSwarm.size(); r++){
        for(int c=0; c<data.columnLength(); c++){
          initialSwarm.get(r).setVariableValue( c, data.get(r, c) );
        }
      }
      ((OMOPSO) algorithm).setInitialSwarm(initialSwarm);
      JMetalLogger.logger.info("Use initial population: "+ FilenameUtils.getName(fileNameOfInitialSolutions));
    }

    // アルゴリズム実行
    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
            .execute() ;
    builder.getSolutionListEvaluator().shutdown();  // マルチスレッドのシャットダウン

    // 探索結果と計算結果の取得
    List<DoubleSolution> population = algorithm.getResult() ;
    long computingTime = algorithmRunner.getComputingTime() ;
    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

    // 評価スレッドの終了処理
    evaluator.shutdown();

    // 最終解の出力
    printFinalSolutionSet(population);
  }
}
