package org.uma.jmetal.runner.multiobjective;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.util.StringUtility;
import org.apache.commons.io.FilenameUtils;
import org.uma.jmetal.algorithm.Algorithm;
import org.uma.jmetal.algorithm.multiobjective.moead.AbstractMOEAD;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADBuilder;
import org.uma.jmetal.algorithm.multiobjective.moead.MOEADBuilder.Variant;
import org.uma.jmetal.algorithm.multiobjective.moead.ParallelConstraintMOEAD;
import org.uma.jmetal.algorithm.multiobjective.moead.ParallelConstraintMOEADWithEpsilonArchive;
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
public class ParallelConstraintMOEADWithEpsilonArchiveRunner extends AbstractAlgorithmRunner {
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
    DifferentialEvolutionCrossover crossover;
    SolutionListEvaluator<DoubleSolution> evaluator ;

    String problemName ;
    String referenceParetoFront = "";
    int numberOfThreads=1;
    String fileNameOfInitialPopulation = "";
    if (args.length == 1) {
      problemName = args[0];
    } else if (args.length == 2) {
      problemName = args[0] ;
      referenceParetoFront = args[1] ;
    }else if (args.length == 3) {
      problemName = args[0] ;
      referenceParetoFront = args[1] ;
      numberOfThreads = Integer.valueOf(args[2]);
    }else if (args.length == 4) {
      problemName = args[0] ;
      referenceParetoFront = args[1] ;
      numberOfThreads = Integer.valueOf(args[2]);
      fileNameOfInitialPopulation = args[3];
    } else {
      //problemName = "org.uma.jmetal.problem.multiobjective.Tanaka";
      //problemName = "org.uma.jmetal.problem.multiobjective.ep.EPZeb2ObjectiveNoConstraint";
      //problemName = "org.uma.jmetal.problem.multiobjective.ep.EPZeb2Objective1ConstraintEachComfortNormalized";
      problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";
      //referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/Tanaka.pf";
      numberOfThreads = 6;
      //fileNameOfInitialPopulation = "C:\\workspace\\jMetal_edit\\initialSolutions.csv";
      //fileNameOfInitialPopulation = "C:\\workspace\\jMetal_edit\\result\\20190323_MOEAD_500世代_設計変数差分_快適度制約あり\\variable30.csv";
    }

    problem = (DoubleProblem)ProblemUtils.<DoubleSolution> loadProblem(problemName);

    if (numberOfThreads == 1) {
      evaluator = new SequentialSolutionListEvaluator<DoubleSolution>() ;
    } else {
//      evaluator = new MultithreadedSolutionListEvaluator<DoubleSolution>(numberOfThreads, problem) ;
      evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>(numberOfThreads, problem) ;
    }

    double cr = 0.9 ; // default: 1.0
    double f = 0.7 ;  // default: 0.5
    crossover = new DifferentialEvolutionCrossover(cr, f, "rand/1/bin");

    double mutationProbability = 1.0 / problem.getNumberOfVariables();
    double mutationDistributionIndex = 20.0;
    mutation = new PolynomialMutation(mutationProbability, mutationDistributionIndex);

    int neighborSize = 5;    // default: 20
    int populationSize = 35;  // default: 300
    int iterations = 500;   // 500
    int maxEvaluations = iterations * populationSize;  // default: 50000

    algorithm = new MOEADBuilder(problem, Variant.ParallelConstraintMOEADWithEpsilonArchive)
            .setCrossover(crossover)
            .setMutation(mutation)
            .setMaxEvaluations(maxEvaluations)
            .setPopulationSize(populationSize)
            .setResultPopulationSize(populationSize)
            .setNeighborhoodSelectionProbability(0.9)
            .setMaximumNumberOfReplacedSolutions(2)
            .setNeighborSize(neighborSize)
            .setFunctionType(AbstractMOEAD.FunctionType.TCHE)
            .setDataDirectory("MOEAD_Weights")
            .setEvaluator(evaluator)
            .build() ;

    // 初期値を設定
    if(!StringUtility.isNullOrEmpty(fileNameOfInitialPopulation)) {
      List<DoubleSolution> initialPopulation = new ArrayList<DoubleSolution>();
      Matrix data = new Matrix(Csv.read(fileNameOfInitialPopulation));
      for (int r = 0; r < populationSize; r++) {
        initialPopulation.add(problem.createSolution());
        for (int c = 0; c < data.columnLength(); c++) {
          initialPopulation.get(r).setVariableValue(c, data.get(r, c));
        }
      }
      ((ParallelConstraintMOEADWithEpsilonArchive)algorithm).setInitialPopulation(initialPopulation);
      JMetalLogger.logger.info("Use initial population: "+ FilenameUtils.getName(fileNameOfInitialPopulation));
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
