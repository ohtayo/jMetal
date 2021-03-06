package org.uma.jmetal.runner.multiobjective;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.StringUtility;
import org.apache.commons.io.FilenameUtils;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAII;
import org.uma.jmetal.algorithm.multiobjective.nsgaii.NSGAIIBuilder;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.SelectionOperator;
import org.uma.jmetal.operator.impl.crossover.SBXCrossover;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.operator.impl.selection.BinaryTournamentSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.*;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.AtOneTimeSolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;

import java.io.FileNotFoundException;
import java.util.List;

/**
 * Class for configuring and running the NSGA-II algorithm (parallel version)
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */

public class ParallelNSGAIIRunner extends AbstractAlgorithmRunner {
  /**
   * @param args Command line arguments.
   * @throws SecurityException
   * Invoking command:
  java org.uma.jmetal.runner.multiobjective.ParallelNSGAIIRunner problemName [referenceFront]
   */
  public static void main(String[] args) throws JMetalException, FileNotFoundException {
    DoubleProblem problem;
    Algorithm<List<DoubleSolution>> algorithm;
    CrossoverOperator<DoubleSolution> crossover;
    MutationOperator<DoubleSolution> mutation;
    SelectionOperator<List<DoubleSolution>, DoubleSolution> selection;

    String problemName;
    int numberOfIndividuals = 35;
    int numberOfGenerations = 500;
    int numberOfThreads = 6;
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
      problemName = "org.uma.jmetal.problem.multiobjective.zdt.ZDT1";
      referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/ZDT1.pf" ;
      problemName = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4";
      referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf" ;
      problemName = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1";
      problemName = "org.uma.jmetal.problem.multiobjective.dtlz.DTLZ1";
      referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf" ;
      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";
    }

    problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);

    double crossoverProbability = 0.9 ;
    double crossoverDistributionIndex = 20.0 ;
    crossover = new SBXCrossover(crossoverProbability, crossoverDistributionIndex) ;

    double mutationProbability = 1.0 / problem.getNumberOfVariables() ;
    double mutationDistributionIndex = 20.0 ;
    mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex) ;

    selection = new BinaryTournamentSelection<DoubleSolution>();

    SolutionListEvaluator<DoubleSolution> evaluator;
    if(problemName.contains("AtOneTime")) {
      evaluator = new AtOneTimeSolutionListEvaluator();
    }else{
      evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>(numberOfThreads, problem);
    }

    int maxEvaluations = numberOfGenerations * numberOfIndividuals;  // default: 25000

    NSGAIIBuilder<DoubleSolution> builder = new NSGAIIBuilder<DoubleSolution>(problem, crossover, mutation, numberOfIndividuals)
        .setSelectionOperator(selection)
        .setMaxEvaluations(maxEvaluations)
        .setSolutionListEvaluator(evaluator) ;

    algorithm = builder.build() ;

    // 初期値のファイル名の指定があれば初期値を設定
    if(!StringUtility.isNullOrEmpty(fileNameOfInitialSolutions)) {
      List<DoubleSolution> initialPopulation = ((NSGAII<DoubleSolution>) algorithm).createInitialPopulation();
      Matrix data = new Matrix(Csv.read(fileNameOfInitialSolutions));
      for (int r = 0; r < initialPopulation.size(); r++) {
        for (int c = 0; c < data.columnLength(); c++) {
          initialPopulation.get(r).setVariableValue(c, data.get(r, c));
        }
      }
      ((NSGAII<DoubleSolution>) algorithm).setInitialPopulation(initialPopulation);
      JMetalLogger.logger.info("Use initial population: "+ FilenameUtils.getName(fileNameOfInitialSolutions));
    }

    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
            .execute() ;

    builder.getSolutionListEvaluator().shutdown();

    List<DoubleSolution> population = algorithm.getResult() ;
    long computingTime = algorithmRunner.getComputingTime() ;

    evaluator.shutdown();

    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

    printFinalSolutionSet(population);
    if (!referenceParetoFront.equals("")) {
      printQualityIndicators(population, referenceParetoFront) ;
    }
  }
}
