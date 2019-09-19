package org.uma.jmetal.runner.multiobjective;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
import jp.ohtayo.commons.math.Vector;
import org.apache.commons.io.FileUtils;
import org.uma.jmetal.algorithm.multiobjective.moead.ParallelConstraintMOEADWithEpsilonArchive;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.qualityindicator.impl.hypervolume.PISAHypervolume;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.ProblemUtils;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.uma.jmetal.util.AbstractAlgorithmRunner;
import org.uma.jmetal.util.front.Front;
import org.uma.jmetal.util.front.imp.ArrayFront;
import org.uma.jmetal.util.front.util.FrontNormalizer;
import org.uma.jmetal.util.front.util.FrontUtils;
import org.uma.jmetal.util.point.PointSolution;

/**
 * Class for running algorithm many times.
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
public class AlgorithmRunnerRepeater {
    public static void main(String[] args) throws JMetalException {
        String resultFolder = "C:\\workspace\\jMetal\\result\\";
        String archiveFolderBase = "C:\\workspace\\jMetal\\archive\\";

        String experimentName = "C3DTLZ1_OMOPSO_pop35_gen2000";
        String problemName =  "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1";
        String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf";

/*
        String experimentName = "C3DTLZ4_OMOPSO_pop35_gen2000";
        String problemName =  "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4";
        String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf";
*/

        DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);
        int generations = 2000;
        int iterations = 2;    // 20
/*
        // empty the result folder
        try {
            FileUtils.cleanDirectory(new File(resultFolder));
        }catch(IOException e){
            e.printStackTrace();
        }

        // run algorithm "iteration" times
        for( int i=0; i<iterations; i++) {
            // run algorithm
            ParallelOMOPSORunner.main(args);
            //    ParallelNSGAIIIWithEpsilonArchiveRunner.main(args);
            //try {
            //    ParallelConstraintMOEADWithEpsilonArchiveRunner.main(args);
            //    //ParallelNSGAIIWithEpsilonArchiveRunner.main(args);
            //}catch(FileNotFoundException e){
            //    e.printStackTrace();
            //}

            // copy result
            String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i)+"\\";
            try {
                FileUtils.copyDirectory(new File(resultFolder), new File(archiveFolder));
            }catch (IOException e){
                e.printStackTrace();
            }
        }
*/
        // (1)すべての試行で得られたすべての結果の最大値と最小値で正規化してHVを求めるように修正する
        // →OK
        // (2)DTLZ1で4目的以上の場合HVが算出できない問題を解決する．invertedfrontがおかしいような気がする．他の結果のinvertedfrontの様子を見てみる．
        // →InvertedFrontは0-1の範囲で1-0にinvertするだけなので，0-1の範囲を超過する解が多いとほとんど0になってしまう．
        // →(1)の最大値最小値で正規化すればよい⇒OK．将来的にすべての探索手法ですべての結果の最大値/最小値で正規化する．
        // Todo: (3)OMOPSOだとC3DTLZ4は100個体1000世代では全然うまく探索できていない．他の手法も同様か試してHVを計算してリファレンスとする．
        // Todo: 各runnerのargsの仕様を共通化して，repeaterから指定できるようにする．
        // problemName, numberOfIndividuals, numberOfIterations, numberOfThreads, nameOfReferenceParetoFront, nameOfInitialSolutions, その他手法固有値
        // (4)hypervolumeの世代ごとの平均値をMatrixに追加して保存するように追加する．
        // -> OK.

        // read objectives and min/max values of all solutions
        Matrix minimumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), generations);
        Matrix maximumValuesInIterate = new Matrix(problem.getNumberOfObjectives(), generations);
        Vector minimumValues = new Vector(problem.getNumberOfObjectives(), Double.MAX_VALUE);
        Vector maximumValues = new Vector(problem.getNumberOfObjectives(), Double.MIN_VALUE);
        Matrix[][] solutions = new Matrix[iterations][generations];
        for( int i=0; i<iterations; i++) {
            for (int g = 0; g < generations; g++) {
                // read fitness in each generation
                String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i) + "\\";
                String solutionsFile = "epsilonFitness" + (g+1) + ".csv";
                solutions[i][g] = new Matrix(Csv.read(archiveFolder + solutionsFile));
                minimumValuesInIterate.setColumn(g, solutions[i][g].min(Matrix.DIRECTION_COLUMN));
                maximumValuesInIterate.setColumn(g, solutions[i][g].max(Matrix.DIRECTION_COLUMN));
            }
            Matrix tempMinimumValues = new Matrix(problem.getNumberOfObjectives(), 2);
            tempMinimumValues.setColumn(0, minimumValues);
            tempMinimumValues.setColumn(1, minimumValuesInIterate.min(Matrix.DIRECTION_ROW));
            minimumValues = tempMinimumValues.min(Matrix.DIRECTION_ROW);
            Matrix tempMaximumValues = new Matrix(problem.getNumberOfObjectives(), 2);
            tempMaximumValues.setColumn(0, maximumValues);
            tempMaximumValues.setColumn(1, maximumValuesInIterate.max(Matrix.DIRECTION_ROW));
            maximumValues = tempMaximumValues.max(Matrix.DIRECTION_ROW);
        }

        // calculate average HyperVolume of each generations
        Matrix normalizedHypervolumes = new Matrix(iterations, generations);
        String hypervolumesFolder = archiveFolderBase + "Hypervolume\\";
        for( int i=0; i<iterations; i++) {
            for (int g = 0; g < generations; g++) {
                //
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
                    FrontNormalizer frontNormalizer = new FrontNormalizer(minimumValues.get(), maximumValues.get());
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
        Matrix averagedHypervolume = new Matrix(1, generations);
        averagedHypervolume.setRow(0, normalizedHypervolumes.mean(Matrix.DIRECTION_COLUMN));

        // save hypervolume
        Csv.write(hypervolumesFolder+"HV(normalized)_"+experimentName+".csv", normalizedHypervolumes.T().get());
        Csv.write(hypervolumesFolder+"HV(averaged)_"+experimentName+".csv"  , averagedHypervolume.T().get());
    }
}
