package org.uma.jmetal.qualityIndicator;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.qualityindicator.QualityIndicator;
import org.uma.jmetal.qualityindicator.impl.*;
import org.uma.jmetal.qualityindicator.impl.hypervolume.PISAHypervolume;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.front.Front;
import org.uma.jmetal.util.front.imp.ArrayFront;
import org.uma.jmetal.util.front.util.FrontNormalizer;
import org.uma.jmetal.util.front.util.FrontUtils;
import org.uma.jmetal.util.point.PointSolution;

import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;

/**
 * Calculate Hypervolume from many search results.
 *
 * @author Ohta Yoshihiro <ohta.yoshihiro@outlook.jp>
 */
public class CalculateIndicatorFromManyResultsRunner {
  public static void main(String[] args) throws Exception {
    // (1)すべての試行で得られたすべての結果の最大値と最小値で正規化してHVを求めるように修正する
    // →OK
    // (2)DTLZ1で4目的以上の場合HVが算出できない問題を解決する．invertedfrontがおかしいような気がする．他の結果のinvertedfrontの様子を見てみる．
    // →InvertedFrontは0-1の範囲で1-0にinvertするだけなので，0-1の範囲を超過する解が多いとほとんど0になってしまう．
    // →(1)の最大値最小値で正規化すればよい⇒OK．将来的にすべての探索手法ですべての結果の最大値/最小値で正規化する．
    // Todo: (3)OMOPSOだとC3DTLZ4は100個体1000世代では全然うまく探索できていない．他の手法も同様か試してHVを計算してリファレンスとする．
    // Todo: 各runnerのargsの仕様を共通化して，repeaterから指定できるようにする．
    // problemName, numberOfIndividuals, numberOfIterations, numberOfThreads, nameOfReferenceParetoFront, nameOfInitialSolutions, その他手法固有値
    // Todo: (4)全problemNameの最大値・最小値を算出して保存するだけにする．
    // Todo: (5)その後，すべてのproblemNameの最大値・最小値で正規化して世代ごとにhypervolume計算するようにする


    // folders
    String resultFolder = "C:\\workspace\\jMetal\\result\\";
    String archiveFolderBase = "C:\\workspace\\jMetal\\archive\\";

    // problem and search settings
    String problemName = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1";
    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf";
//    String problem = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf";

    String[] algorithms = {
        "ParallelOMOPSO",
        "ParallelNSGAIIWithEpsilonArchive",
        "ParallelNSGAIIIWithEpsilonArchive",
        "ParallelConstraintMOEADWithEpsilonArchive"
    };
    int numberOfIndividuals = 35;
    int numberOfGenerations = 2000;
    int numberOfRepeats = 20;    // 20
    int numberOfThreads = 4;

    // 目的関数の定義
    DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);


    Matrix minimumValues = new Matrix(problem.getNumberOfObjectives(), algorithms.length, Double.MAX_VALUE);
    Matrix maximumValues = new Matrix(problem.getNumberOfObjectives(), algorithms.length, Double.MIN_VALUE);
    for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
      String[] tempProblemName = problemName.split(Pattern.quote("."));
      String algorithmName = algorithms[algorithmNumber];
      String experimentName = tempProblemName[tempProblemName.length - 1] + "_"+algorithmName+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;

      // read objectives and min/max values of all solutions
      Matrix minimumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), numberOfGenerations);
      Matrix maximumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), numberOfGenerations);
      Matrix[][] solutions = new Matrix[numberOfRepeats][numberOfGenerations];
      for (int i = 0; i < numberOfRepeats; i++) {
        for (int g = 0; g < numberOfGenerations; g++) {
          // read fitness in each generation
          String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i) + "\\";
          String solutionsFile = "epsilonFitness" + (g + 1) + ".csv";
          solutions[i][g] = new Matrix(Csv.read(archiveFolder + solutionsFile));
          minimumValuesInIterate.setColumn(g, solutions[i][g].min(Matrix.DIRECTION_COLUMN));
          maximumValuesInIterate.setColumn(g, solutions[i][g].max(Matrix.DIRECTION_COLUMN));
        }
        Matrix tempMinimumValues = new Matrix(problem.getNumberOfObjectives(), 2);
        tempMinimumValues.setColumn(0, minimumValues.getColumn(algorithmNumber));
        tempMinimumValues.setColumn(1, minimumValuesInIterate.min(Matrix.DIRECTION_ROW));
        minimumValues.setColumn(algorithmNumber, tempMinimumValues.min(Matrix.DIRECTION_ROW));
        Matrix tempMaximumValues = new Matrix(problem.getNumberOfObjectives(), 2);
        tempMaximumValues.setColumn(0, maximumValues.getColumn(algorithmNumber));
        tempMaximumValues.setColumn(1, maximumValuesInIterate.max(Matrix.DIRECTION_ROW));
        maximumValues.setColumn(algorithmNumber, tempMaximumValues.max(Matrix.DIRECTION_ROW));
      }
    }

    // calculate average HyperVolume of each generations
    Matrix normalizedHypervolumes = new Matrix(numberOfRepeats, numberOfGenerations);
    Matrix averagedHypervolumes = new Matrix(algorithms.length, numberOfGenerations);
    String hypervolumesFolder = archiveFolderBase + "Hypervolume\\";
    Matrix[][] solutions = new Matrix[numberOfRepeats][numberOfGenerations];

    for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
      String[] tempProblemName = problemName.split(Pattern.quote("."));
      String algorithmName = algorithms[algorithmNumber];
      String experimentName = tempProblemName[tempProblemName.length - 1] + "_" + algorithmName + "_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;

      for (int i = 0; i < numberOfRepeats; i++) {
        for (int g = 0; g < numberOfGenerations; g++) {
          String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i) + "\\";
          String solutionsFile = "epsilonFitness" + (g + 1) + ".csv";
          solutions[i][g] = new Matrix(Csv.read(archiveFolder + solutionsFile));
          List<DoubleSolution> population = new ArrayList<DoubleSolution>(solutions[i][g].length());
          for (int s = 0; s < solutions[i][g].length(); s++) {
            DoubleSolution newIndividual = problem.createSolution();
            for (int o = 0; o < solutions[i][g].columnLength(); o++) {
              newIndividual.setObjective(o, solutions[i][g].get(s, o));
            }
            population.add(newIndividual);
          }

          // calculate hypervolume
          try {
            Front referenceFront = new ArrayFront(referenceParetoFront);
            FrontNormalizer frontNormalizer = new FrontNormalizer(minimumValues.min(Matrix.DIRECTION_ROW).get(), maximumValues.max(Matrix.DIRECTION_ROW).get());
            Front normalizedReferenceFront = frontNormalizer.normalize(referenceFront);
            Front normalizedFront = frontNormalizer.normalize(new ArrayFront(population));
            List<PointSolution> normalizedPopulation = FrontUtils.convertFrontToSolutionList(normalizedFront);
            double normalizedHypervolume = new PISAHypervolume<PointSolution>(normalizedReferenceFront).evaluate(normalizedPopulation);
            normalizedHypervolumes.set(i, g, normalizedHypervolume);
          } catch (FileNotFoundException e) {
            e.printStackTrace();
          }
        }
      }
      // hypervolumeの世代ごとの平均値をMatrixに追加
      averagedHypervolumes.setRow(algorithmNumber, normalizedHypervolumes.mean(Matrix.DIRECTION_COLUMN));

      // save hypervolume
      Csv.write(hypervolumesFolder+"HV(normalized)_"+experimentName+".csv", normalizedHypervolumes.T().get());
    }
    Csv.write(hypervolumesFolder+"HV(averaged)_"+problemName+".csv"  , averagedHypervolumes.T().get());
  }
}
