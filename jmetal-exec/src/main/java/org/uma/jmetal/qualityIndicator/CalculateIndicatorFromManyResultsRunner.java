package org.uma.jmetal.qualityIndicator;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.log.Logging;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.apache.commons.io.FileUtils;
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

import java.io.File;
import java.io.FileNotFoundException;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.regex.Pattern;

/**
 * Calculate Hypervolume from many search results.
 *
 * @author Ohta Yoshihiro <ohta.yoshihiro@outlook.jp>
 */
public class CalculateIndicatorFromManyResultsRunner {
  public static void main(String[] args)
  {
    new CalculateIndicatorFromManyResultsRunner().calculate(args);
  }

  public void calculate(String[] args) {

    // folders
    String archiveFolderBase = "C:\\workspace\\jMetal\\archive\\";

    // problem and search settings
//    String problemName = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_S_DTLZ2";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.newDtlz.C3_RosenbrockDTLZ2";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_R_DTLZ2";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_S_DTLZ3";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ3.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.UF.UF11";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.UF.UF12";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
//    String problemName = "org.uma.jmetal.problem.multiobjective.maf.C3_MaF02";
//    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";
    String problemName = "org.uma.jmetal.problem.multiobjective.coco.C_RosenbrockRastrigin";
    String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf";

    String[] algorithms = {
        "ParallelNSGAII",
        "ParallelNSGAIIWithEpsilonArchive",
        "ParallelNSGAIII",
        "ParallelNSGAIIIWithEpsilonArchive",
        "ParallelConstraintMOEAD",
        "ParallelConstraintMOEADWithEpsilonArchive",
        "ParallelConstraintMOEADDEWithEpsilonArchive",
        "ParallelOMOPSO",
        "ParallelOMOPSOWithSizeLimitedArchive",
//      "ParallelDirectionalOMOPSOWithSizeLimitedArchive",
    };

    // definition of problem
    int numberOfIndividuals = 35;
    int numberOfGenerations = 500; // 2000
    int numberOfRepeats = 5;    // 20
    int numberOfThreads = 6;

    DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);
    if( problem.getNumberOfObjectives()==3 || problem.getNumberOfObjectives()==8 ) {
      numberOfIndividuals = 36;
    }

    calculateExtremeValues(archiveFolderBase, problem, algorithms, numberOfIndividuals, numberOfGenerations, numberOfRepeats, numberOfThreads);
    calculateHypervolumes(archiveFolderBase, problem, algorithms, numberOfIndividuals, numberOfGenerations, numberOfRepeats, numberOfThreads);

  }

  private String getFitnessFileName(String algorithmName, int generation)
  {
    String fileName = "fitness"+(generation+1) +".csv";
    if(algorithmName.contains("Archive")){
      fileName = "truncated" + fileName;
    }else if(algorithmName.contains("OMOPSO")){
      fileName = "epsilon" + fileName;
    }
    return fileName;
  }

  private void calculateExtremeValues(String archiveFolderBase, DoubleProblem  problem, String[] algorithms,
                                      int numberOfIndividuals, int numberOfGenerations, int numberOfRepeats, int numberOfThreads){
    // calculate minimum and maximum objective values from search results.
    JMetalLogger.logger.info("Calculate minimum and maximum objective values from search results.");

    Matrix minimumValues = new Matrix(problem.getNumberOfObjectives(), algorithms.length, Double.MAX_VALUE);
    Matrix maximumValues = new Matrix(problem.getNumberOfObjectives(), algorithms.length, Double.MIN_VALUE);

    // アルゴリズム毎に最小値・最大値を計算する．計算結果はcsvで保存しておき，csvがあればそれを読み込む．
    for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
      String algorithmName = algorithms[algorithmNumber];
      String experimentName = problem.getName() + "_"+algorithmName+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;
      JMetalLogger.logger.info(experimentName);

      Matrix minimumValuesOfAlgorithm = new Matrix(problem.getNumberOfObjectives(), numberOfRepeats);
      Matrix maximumValuesOfAlgorithm = new Matrix(problem.getNumberOfObjectives(), numberOfRepeats);
      if(new File(archiveFolderBase+experimentName+"\\minimumValues.csv").exists() ){
        JMetalLogger.logger.info("Extreme values are already calculated. Read calculated values.");
        minimumValuesOfAlgorithm = new Matrix(Csv.read(archiveFolderBase + experimentName + "\\minimumValues.csv"));
        maximumValuesOfAlgorithm = new Matrix(Csv.read(archiveFolderBase + experimentName + "\\maximumValues.csv"));
      }else {
        // calculate extreme values of each iteration using multi threads.
        calculateExtremeValuesInIterate(minimumValuesOfAlgorithm, maximumValuesOfAlgorithm, problem, numberOfThreads, numberOfGenerations, archiveFolderBase, experimentName, algorithmName);

        Csv.write(archiveFolderBase + experimentName + "\\minimumValues.csv", minimumValuesOfAlgorithm.min(Matrix.DIRECTION_ROW).get());
        Csv.write(archiveFolderBase + experimentName + "\\maximumValues.csv", maximumValuesOfAlgorithm.max(Matrix.DIRECTION_ROW).get());
      }
      minimumValues.setColumn(algorithmNumber, minimumValuesOfAlgorithm.min(Matrix.DIRECTION_ROW));
      maximumValues.setColumn(algorithmNumber, maximumValuesOfAlgorithm.min(Matrix.DIRECTION_ROW));
    }
    // 問題に対するすべてのアルゴリズムの最大値・最小値のセットを保存
    String extremeValueFolder = archiveFolderBase + problem.getName() +"\\";
    new File(extremeValueFolder).mkdir();
    Csv.write(extremeValueFolder+"minimumValue.csv", minimumValues.get(), String.join(",",algorithms));
    Csv.write(extremeValueFolder+"maximumValue.csv", maximumValues.get(), String.join(",",algorithms));
  }

  private void calculateExtremeValuesInIterate(Matrix minimumValuesOfAlgorithm, Matrix maximumValuesOfAlgorithm, DoubleProblem problem,
                                               int numberOfThreads, int numberOfGenerations,
                                               String archiveFolderBase, String experimentName, String algorithmName)
  {
    int numberOfRepeats = minimumValuesOfAlgorithm.columnLength();
    //8スレッドの枠を用意
    ExecutorService executor = Executors.newFixedThreadPool(numberOfThreads);
    try{
      //別スレッドで順次解の評価タスクを渡す
      for(int i=0; i<numberOfRepeats; i++)
      {
        executor.execute(new CalculateExtremeValuesOneIteration(minimumValuesOfAlgorithm, maximumValuesOfAlgorithm, problem,
            i, numberOfGenerations,
            archiveFolderBase, experimentName, algorithmName));
      }
    }finally{
      //新規タスクの受付を終了して残ったタスクを継続する．
      executor.shutdown();
      try {
        //指定時間(個体数×1分)が経過するか，全タスクが終了するまで処理を停止する．
        executor.awaitTermination(numberOfRepeats*100, TimeUnit.MINUTES);
      } catch (InterruptedException e) {
        e.printStackTrace();
        JMetalLogger.logger.severe(e.getMessage());
      }
    }
  }

  private class CalculateExtremeValuesOneIteration implements Runnable{

    private Matrix minimumValuesOfAlgorithm;
    private Matrix maximumValuesOfAlgorithm;
    private int repeats;
    private DoubleProblem problem;
    private int numberOfGenerations;
    private String archiveFolderBase;
    private String experimentName;
    private String algorithmName;
    //コンストラクタ
    public CalculateExtremeValuesOneIteration(Matrix minimumValuesOfAlgorithm, Matrix maximumValuesOfAlgorithm, DoubleProblem problem,
                                              int repeats, int numberOfGenerations,
                                              String archiveFolderBase, String experimentName, String algorithmName){
      this.minimumValuesOfAlgorithm = minimumValuesOfAlgorithm;
      this.maximumValuesOfAlgorithm = maximumValuesOfAlgorithm;
      this.repeats = repeats;
      this.problem = problem;
      this.numberOfGenerations = numberOfGenerations;
      this.archiveFolderBase = archiveFolderBase;
      this.experimentName = experimentName;
      this.algorithmName = algorithmName;
    }

    //実行
    public void run(){
      JMetalLogger.logger.info("Repeats: " + repeats);

      Matrix minimumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), numberOfGenerations);
      Matrix maximumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), numberOfGenerations);
      for (int g = 0; g < numberOfGenerations; g++) {
        //JMetalLogger.logger.info("Generations: "+g);
        // read fitness in each generation
        String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(repeats) + "\\";
        Matrix fitness = new Matrix(Csv.read(archiveFolder + getFitnessFileName(algorithmName, g)));
        minimumValuesInIterate.setColumn(g, fitness.min(Matrix.DIRECTION_COLUMN));
        maximumValuesInIterate.setColumn(g, fitness.max(Matrix.DIRECTION_COLUMN));
      }
      minimumValuesOfAlgorithm.setColumn(repeats, minimumValuesInIterate.min(Matrix.DIRECTION_ROW));
      maximumValuesOfAlgorithm.setColumn(repeats, maximumValuesInIterate.max(Matrix.DIRECTION_ROW));
    }
  }


  // calculate average HyperVolume of each generations
  private void calculateHypervolumes(String archiveFolderBase, DoubleProblem problem, String[] algorithms,
                                     int numberOfIndividuals, int numberOfGenerations, int numberOfRepeats,
                                     int numberOfThreads){
    JMetalLogger.logger.info("Calculate average HyperVolume of each generations.");

    Matrix normalizedHypervolumes = new Matrix(numberOfGenerations, numberOfRepeats);
    Matrix averagedHypervolumes = new Matrix(numberOfGenerations, algorithms.length);
    String hypervolumesFolder = archiveFolderBase + "Hypervolume\\";
    new File(hypervolumesFolder).mkdir();

    for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
      String algorithmName = algorithms[algorithmNumber];
      String experimentName = problem.getName() + "_" + algorithmName + "_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;
      JMetalLogger.logger.info(experimentName);

      if(new File(hypervolumesFolder+"HV(normalized)_"+experimentName+".csv").exists()){
        JMetalLogger.logger.info("Normalized Hypervolume values are already calculated. Read calculated values.");
        normalizedHypervolumes = new Matrix(Csv.read(hypervolumesFolder+"HV(normalized)_"+experimentName+".csv"));
      }else {
        calculateHypervolumesInIterate(normalizedHypervolumes, problem,
            numberOfThreads, numberOfGenerations,
            archiveFolderBase, experimentName, algorithmName);
        // save values
        Csv.write(hypervolumesFolder+"HV(normalized)_"+experimentName+".csv", normalizedHypervolumes.get());
      }
      averagedHypervolumes.setColumn(algorithmNumber, normalizedHypervolumes.mean(Matrix.DIRECTION_ROW));
    }
    Csv.write(hypervolumesFolder+"HV(averaged)_"+ problem.getName()+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations+".csv"  , averagedHypervolumes.get(), String.join(",", algorithms));
  }


  private void calculateHypervolumesInIterate(Matrix normalizedHypervolumes, DoubleProblem problem,
                                              int numberOfThreads, int numberOfGenerations,
                                              String archiveFolderBase, String experimentName, String algorithmName)
  {
    int numberOfRepeats = normalizedHypervolumes.columnLength();
    //8スレッドの枠を用意
    ExecutorService executor = Executors.newFixedThreadPool(numberOfThreads);
    try{
      //別スレッドで順次解の評価タスクを渡す
      for(int i=0; i<numberOfRepeats; i++)
      {
        executor.execute(new CalculateHypervoluemesOneIteration(normalizedHypervolumes, problem,
            i, numberOfGenerations,
            archiveFolderBase, experimentName, algorithmName));
      }
    }finally{
      //新規タスクの受付を終了して残ったタスクを継続する．
      executor.shutdown();
      try {
        //指定時間(個体数×1分)が経過するか，全タスクが終了するまで処理を停止する．
        executor.awaitTermination(numberOfRepeats*100, TimeUnit.MINUTES);
      } catch (InterruptedException e) {
        e.printStackTrace();
        JMetalLogger.logger.severe(e.getMessage());
      }
    }
  }

  private class CalculateHypervoluemesOneIteration implements Runnable{
    private Matrix normalizedHypervolumes;
    private int repeats;
    private DoubleProblem problem;
    private int numberOfGenerations;
    private String archiveFolderBase;
    private String experimentName;
    private String algorithmName;
    //コンストラクタ
    public CalculateHypervoluemesOneIteration(Matrix normalizedHypervolumes, DoubleProblem problem,
                                              int repeats, int numberOfGenerations,
                                              String archiveFolderBase, String experimentName, String algorithmName){
      this.normalizedHypervolumes = normalizedHypervolumes;
      this.repeats = repeats;
      this.problem = problem;
      this.numberOfGenerations = numberOfGenerations;
      this.archiveFolderBase = archiveFolderBase;
      this.experimentName = experimentName;
      this.algorithmName = algorithmName;
    }

    //実行
    public void run(){
      JMetalLogger.logger.info("Repeats: "+repeats);

      String extremeValueFolder = archiveFolderBase + problem.getName() +"\\";
      Matrix minimumValues = new Matrix(Csv.read(extremeValueFolder+"minimumValue.csv", 1, 0));
      Matrix maximumValues = new Matrix(Csv.read(extremeValueFolder+"maximumValue.csv", 1, 0));

      for (int g = 0; g < numberOfGenerations; g++) {
        //JMetalLogger.logger.info("Generations: "+g);
        String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(repeats) + "\\";
        Matrix fitness = new Matrix(Csv.read(archiveFolder + getFitnessFileName(algorithmName, g)));
        List<DoubleSolution> population = new ArrayList<DoubleSolution>(fitness.length());
        for (int s = 0; s < fitness.length(); s++) {
          DoubleSolution newIndividual = problem.createSolution();
          for (int o = 0; o < fitness.columnLength(); o++) {
            newIndividual.setObjective(o, fitness.get(s, o));
          }
          population.add(newIndividual);
        }

        // calculate hypervolume
        FrontNormalizer frontNormalizer = new FrontNormalizer(minimumValues.min(Matrix.DIRECTION_ROW).get(), maximumValues.max(Matrix.DIRECTION_ROW).get());
        Front normalizedFront = frontNormalizer.normalize(new ArrayFront(population));
        List<PointSolution> normalizedPopulation = FrontUtils.convertFrontToSolutionList(normalizedFront);
        int numberOfObjectives = fitness.columnLength();
        double normalizedHypervolume = new PISAHypervolume<PointSolution>().evaluate(normalizedPopulation, numberOfObjectives);
        normalizedHypervolumes.set(g, repeats, normalizedHypervolume);
      }
    }
  }
}
