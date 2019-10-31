package org.uma.jmetal.qualityIndicator;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.uma.jmetal.algorithm.multiobjective.spea2.util.EnvironmentalSelection;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.qualityindicator.QualityIndicator;
import org.uma.jmetal.qualityindicator.impl.*;
import org.uma.jmetal.qualityindicator.impl.hypervolume.PISAHypervolume;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.impl.DefaultDoubleSolution;
import org.uma.jmetal.solution.impl.ParticleSwarmSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.ProblemUtils;
import org.uma.jmetal.util.archive.impl.NonDominatedSolutionListArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.comparator.StrengthFitnessComparator;
import org.uma.jmetal.util.front.Front;
import org.uma.jmetal.util.front.imp.ArrayFront;
import org.uma.jmetal.util.front.util.FrontNormalizer;
import org.uma.jmetal.util.front.util.FrontUtils;
import org.uma.jmetal.util.point.PointSolution;
import org.uma.jmetal.util.solutionattribute.impl.StrengthRawFitness;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

/**
 *
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
public class CalculateTruncatedArchiveRunner {
  public static void main(String[] args) throws Exception
  {
    new CalculateTruncatedArchiveRunner().calculate(args);
  }

  public void calculate(String[] args) throws Exception {
    // folders
    String archiveFolderBase = "C:\\workspace\\jMetal\\archive\\";

    // problem and search settings
    String problemName = "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff4ObjRegretConPMV";

    String[] algorithms = {
        "ParallelOMOPSO",
        "ParallelNSGAIIWithEpsilonArchive",
        "ParallelNSGAIIIWithEpsilonArchive",
        "ParallelConstraintMOEADWithEpsilonArchive",
        "ParallelNSGAII",
        "ParallelNSGAIII",
        "ParallelConstraintMOEAD",
//        "ParallelOMOPSOWithSizeLimitedArchive"
    };
    int numberOfIndividuals = 35;
    int numberOfGenerations = 500; // 2000
    int numberOfRepeats = 1;    // 20
    int numberOfThreads = 4;
    int archiveSize = 400;
    double eta = 0.0075;

    // definition of problem
    DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);

    calculateTruncatedArchive(archiveFolderBase, problem, algorithms, numberOfIndividuals, numberOfGenerations, numberOfRepeats, numberOfThreads, archiveSize, eta);

  }

  private String getFitnessFileName(String algorithmName, int generation)
  {
    String fileName = "fitness"+(generation+1) +".csv";
    if( algorithmName.contains("Archive") || algorithmName.contains("OMOPSO") ){
      fileName = "epsilon" + fileName;
    }
    return fileName;
  }

  private void calculateTruncatedArchive(String archiveFolderBase, DoubleProblem  problem, String[] algorithms,
                                      int numberOfIndividuals, int numberOfGenerations, int numberOfRepeats, int numberOfThreads, int archiveSize, double eta)
  {
    // calculate minimum and maximum objective values from search results.
    JMetalLogger.logger.info("Calculate truncated archive from searched solution of epsilon archive.");

    // アルゴリズム毎に最小値・最大値を計算する．計算結果はcsvで保存しておき，csvがあればそれを読み込む．
    for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
      String algorithmName = algorithms[algorithmNumber];
      String experimentName = problem.getName() + "_"+algorithmName+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;
      JMetalLogger.logger.info(experimentName);

      for(int repeats = 0; repeats < numberOfRepeats; repeats++) {
        JMetalLogger.logger.info("repeat: "+repeats);
        calculateTruncatedArchiveInGeneration(problem, repeats, archiveSize, eta, numberOfThreads, numberOfGenerations, archiveFolderBase, experimentName, algorithmName);
      }
    }
  }

  private void calculateTruncatedArchiveInGeneration(DoubleProblem problem, int repeats, int archiveSize, double eta,
                                               int numberOfThreads, int numberOfGenerations,
                                               String archiveFolderBase, String experimentName, String algorithmName)
  {
    //8スレッドの枠を用意
    ExecutorService executor = Executors.newFixedThreadPool(numberOfThreads);
    try{
      //別スレッドで順次解の評価タスクを渡す
      for(int i=0; i<numberOfGenerations; i++)
      {
        executor.execute(new CalculateTruncatedArchiveRunner.CalculateTruncatedArchiveOneGeneration(
            problem, repeats, i, archiveSize, eta, archiveFolderBase, experimentName, algorithmName));
      }
    }finally{
      //新規タスクの受付を終了して残ったタスクを継続する．
      executor.shutdown();
      try {
        //指定時間(個体数×1分)が経過するか，全タスクが終了するまで処理を停止する．
        executor.awaitTermination(numberOfGenerations*1, TimeUnit.HOURS);
      } catch (InterruptedException e) {
        e.printStackTrace();
        JMetalLogger.logger.severe(e.getMessage());
      }
    }
  }

  private class CalculateTruncatedArchiveOneGeneration implements Runnable{

    private int repeats;
    private DoubleProblem problem;
    private int generation;
    private int archiveSize;
    private double eta;
    private String archiveFolderBase;
    private String experimentName;
    private String algorithmName;
    //コンストラクタ
    public CalculateTruncatedArchiveOneGeneration(DoubleProblem problem, int repeats, int generation, int archiveSize, double eta,
                                              String archiveFolderBase, String experimentName, String algorithmName){
      this.repeats = repeats;
      this.generation = generation;
      this.archiveSize = archiveSize;
      this.eta = eta;
      this.problem = problem;
      this.archiveFolderBase = archiveFolderBase;
      this.experimentName = experimentName;
      this.algorithmName = algorithmName;
    }

    //実行
    public void run(){
      // read fitness in each generation
      String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(repeats) + "\\";
      String archiveFile = archiveFolder + getFitnessFileName(algorithmName, generation);
      String truncatedArchiveFile = archiveFolder + "truncatedfitness"+(generation+1)+".csv";
      if( new File(truncatedArchiveFile).exists() ){
        JMetalLogger.logger.info(truncatedArchiveFile +" is exists.");
      }else {
        Matrix epsilonArchive = new Matrix(Csv.read(archiveFile));
        JMetalLogger.logger.info(archiveFile);

        NonDominatedSolutionListArchive<DoubleSolution> temporaryArchive = new NonDominatedSolutionListArchive<DoubleSolution>(new DominanceComparator<DoubleSolution>(eta));
        // create and add solution to temporary archive.
        for (int s = 0; s < epsilonArchive.length(); s++) {
          // make solution include speed.
          DoubleSolution solution = new DefaultDoubleSolution(problem);
          for (int o = 0; o < epsilonArchive.columnLength(); o++) {
            solution.setObjective(o, epsilonArchive.get(s, o));
          }
          temporaryArchive.add( (DoubleSolution) solution.copy() );
        }
        if(temporaryArchive.size() == 1){
          temporaryArchive.add( (DoubleSolution) temporaryArchive.get(0).copy() );
        }

        // calculate strength fitness and truncate.
        StrengthRawFitness<DoubleSolution> strengthRawFitness = new StrengthRawFitness<DoubleSolution>();
        EnvironmentalSelection<DoubleSolution> environmentalSelection = new EnvironmentalSelection<DoubleSolution>(archiveSize);
        strengthRawFitness.computeDensityEstimator(temporaryArchive.getSolutionList());
        List<DoubleSolution> truncatedArchive = environmentalSelection.execute(temporaryArchive.getSolutionList());

        // save truncated archive to csv.
        Matrix archive = new Matrix(truncatedArchive.size(), epsilonArchive.columnLength());
        for (int s = 0; s < truncatedArchive.size(); s++) {
          archive.setRow(s, new Vector(truncatedArchive.get(s).getObjectives()));
        }
        Csv.write(truncatedArchiveFile, archive.get());
      }
    }
  }


}

