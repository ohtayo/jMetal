package org.uma.jmetal.utility;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff2ObjConPMVUDPAdjusted;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.evaluator.impl.ThreadPoolSolutionListEvaluator;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 *
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
public class EvaluateSolutionsFromFile {
  public static void main(String[] args) throws Exception
  {
    // settings
    String variableFolder = "C:\\workspace\\jMetal\\";
//    String variableFile = "epsilonvariable0_initial.csv";
//    String variableFile = "epsilonvariable500_simulate2obj.csv";
    String variableFile = "epsilonvariable500_DOMOPSO_simulate2obj.csv";
//    String variableFile = "epsilonVariable500_simulate2obj_CEC2019.csv";
//    String variableFile = "epsilonvariable500_simulate4obj.csv";
//    String variableFile = "epsilonvariable500_surrogate2obj.csv";
//    String variableFile = "epsilonvariable500_surrogate2obj_2.csv";
//    String variableFile = "epsilonvariable500_surrogate4obj.csv";

    String fitnessFolder = "C:\\workspace\\jMetal\\";
//    String fitnessFile = "epsilonvariable0_initial.csv_sim4obj.csv";
//    String fitnessFile = "epsilonvariable0_initial.csv_lstm4obj.csv";
//    String fitnessFile = "epsilonvariable500_simulate2obj_CEC2019.csv_sim4obj.csv";
//    String fitnessFile = "epsilonvariable500_simulate2obj_CEC2019.csv_sim4obj_updown.csv";
//    String fitnessFile = "epsilonvariable500_simulate2obj.csv_lstm4obj.csv";
//    String fitnessFile = "epsilonvariable500_simulate4obj.csv_sim4obj.csv";
//    String fitnessFile = "epsilonvariable500_simulate4obj.csv_lstm4obj.csv";
//    String fitnessFile = "epsilonvariable500_surrogate4obj.csv_lstm4obj.csv";
//    String fitnessFile = "epsilonvariable500_surrogate4obj.csv_sim4obj.csv";
//    String fitnessFile = "epsilonvariable500_surrogate2obj.csv_sim4obj.csv";
//    String fitnessFile = "epsilonvariable500_surrogate2obj.csv_lstm4obj.csv";
//    String fitnessFile = "epsilonvariable500_simulate2objupward.csv";
//    String fitnessFile = "epsilonvariable500_simulate4objupward.csv";
//    String fitnessFile = "epsilonvariable500_surrogate2obj_2.csv_lstm2obj.csv";
//    String fitnessFile = "epsilonvariable500_surrogate2obj_2.csv_sim2obj.csv";
    String fitnessFile = "epsilonvariable500_DOMOPSO_simlate2obj.csv_sim2obj.csv";

    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff2ObjConPMVCondLSTM";
//    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff2ObjConPMVUDPAdjusted";
//    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMVCondLSTM";
//  String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff4ObjRegretConPMVUDPAdjusted";
//    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMVInManyPatterns";
//    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";

    int numberOfThreads = 1;

    // definition of problem
    DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);

    // change properties of problem
//    ((ZEBRefModelLSTMVarDiff2ObjConPMVUDPAdjusted) problem).propertiesFile = "lstm2.properties";
//    ((ZEBRefModelLSTMVarDiff2ObjConPMVUDPAdjusted) problem).loadProperties();

    SolutionListEvaluator<DoubleSolution> evaluator = new ThreadPoolSolutionListEvaluator<DoubleSolution>(numberOfThreads, problem);

    // read variables from file
    Matrix variables = new Matrix(Csv.read(variableFolder + variableFile ));

    // create solutions
    int numberOfSolutions = variables.length();
    List<DoubleSolution> solutions = new ArrayList<>();
    for (int s=0; s<numberOfSolutions; s++) {
      DoubleSolution newSolution = problem.createSolution();
      for(int v=0; v<problem.getNumberOfVariables(); v++){
        newSolution.setVariableValue(v, variables.getRow(s).get(v));
      }
      solutions.add(newSolution);
    }

    // evaluation
    List<DoubleSolution> evaluatedSolutions = evaluator.evaluate(solutions, problem);

    // save fitness to file
    Matrix fitness = new Matrix(numberOfSolutions, problem.getNumberOfObjectives());
    for(int s=0; s<numberOfSolutions; s++) {
      fitness.setRow(s, new Vector(evaluatedSolutions.get(s).getObjectives()));
    }
    Csv.write(fitnessFolder + fitnessFile, fitness.get());
  }
}

