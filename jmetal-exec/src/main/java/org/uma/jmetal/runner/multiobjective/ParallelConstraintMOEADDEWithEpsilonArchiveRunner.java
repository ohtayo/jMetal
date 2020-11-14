package org.uma.jmetal.runner.multiobjective;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.StringUtility;
import org.apache.commons.io.FilenameUtils;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.moead.AbstractMOEAD;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADBuilder;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADBuilder.Variant;
import org.uma.jmetal.algorithm.multiobjective.moead.ParallelConstraintMOEADWithEpsilonArchive;
import org.uma.jmetal.operator.CrossoverOperator;
import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.crossover.DifferentialEvolutionCrossover;
import org.uma.jmetal.operator.impl.mutation.PolynomialMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.AlgorithmRunner;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.AtOneTimeSolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.SequentialSolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;

/**
 * Class for configuring and running the MOEA/D algorithm
 *
 * @author Antonio J. Nebro <antonio@lcc.uma.es>
 */
public class ParallelConstraintMOEADDEWithEpsilonArchiveRunner extends AbstractAlgorithmRunner {
  /**
   * @param args Command line arguments.
   * @throws SecurityException
   * Invoking command:
  java org.uma.jmetal.runner.multiobjective.MOEADRunner problemName [referenceFront]
   */
  public static void main(String[] args) throws FileNotFoundException {
    DoubleProblem problem;
    Algorithm<List<DoubleSolution>> algorithm;
    MutationOperator<DoubleSolution> mutation;
    CrossoverOperator<DoubleSolution> crossover;

    String problemName ;
    int numberOfIndividuals = 36;  // default: 300
    int numberOfGenerations = 1000;   // 500
    int numberOfThreads=1;
    String referenceParetoFront = "";
    String fileNameOfInitialSolutions = "";
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
      numberOfThreads = 6;
      problemName = "org.uma.jmetal.problem.multiobjective.UF.C3_UF11";
      problemName = "org.uma.jmetal.problem.multiobjective.maf.C3_MaF02";
      referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
    }

    int archiveSize = 400;

    problem = (DoubleProblem)ProblemUtils.<DoubleSolution> loadProblem(problemName);

    // マルチスレッドの評価クラスの設定
    SolutionListEvaluator<DoubleSolution> evaluator;
    if(problemName.contains("AtOneTime")) {
      evaluator = new AtOneTimeSolutionListEvaluator();
    }else{
      evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>(numberOfThreads, problem);
    }

    double cr = 0.9 ; // default: 1.0
    double f = 0.7 ;  // default: 0.5
    crossover = new DifferentialEvolutionCrossover(cr, f, "rand/1/bin");

    double mutationProbability = 1.0 / problem.getNumberOfVariables();
    double mutationDistributionIndex = 20.0;
    mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    int neighborSize = 5;    // default: 20
    int maxEvaluations = numberOfGenerations * numberOfIndividuals;  // default: 50000

    algorithm = new MOEADBuilder(problem, Variant.ParallelConstraintMOEADDEWithEpsilonArchive)
            .setCrossover(crossover)
            .setMutation(mutation)
            .setMaxEvaluations(maxEvaluations)
            .setPopulationSize(numberOfIndividuals)
            .setResultPopulationSize(numberOfIndividuals)
            .setArchiveSize(archiveSize)
            .setNeighborhoodSelectionProbability(0.9)
            .setMaximumNumberOfReplacedSolutions(2)
            .setNeighborSize(neighborSize)
            .setFunctionType(AbstractMOEAD.FunctionType.TCHE)
            .setDataDirectory("MOEAD_Weights")
            .setEvaluator(evaluator)
            .build() ;

    // 初期値を設定
    if(!StringUtility.isNullOrEmpty(fileNameOfInitialSolutions)) {
      List<DoubleSolution> initialPopulation = new ArrayList<DoubleSolution>();
      Matrix data = new Matrix(Csv.read(fileNameOfInitialSolutions));
      for (int r = 0; r < numberOfIndividuals; r++) {
        initialPopulation.add(problem.createSolution());
        for (int c = 0; c < data.columnLength(); c++) {
          initialPopulation.get(r).setVariableValue(c, data.get(r, c));
        }
      }
      ((ParallelConstraintMOEADWithEpsilonArchive)algorithm).setInitialPopulation(initialPopulation);
      JMetalLogger.logger.info("Use initial population: "+ FilenameUtils.getName(fileNameOfInitialSolutions));
    }

    AlgorithmRunner algorithmRunner = new AlgorithmRunner.Executor(algorithm)
        .execute() ;

    List<DoubleSolution> population = algorithm.getResult() ;
    long computingTime = algorithmRunner.getComputingTime() ;

    JMetalLogger.logger.info("Total execution time: " + computingTime + "ms");

    printFinalSolutionSet(population);
    if (!referenceParetoFront.equals("")) {
      printQualityIndicators(population, referenceParetoFront) ;
    }
  }
}
