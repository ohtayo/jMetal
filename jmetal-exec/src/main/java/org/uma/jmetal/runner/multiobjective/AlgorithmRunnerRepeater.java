package org.uma.jmetal.runner.multiobjective;

import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.math.Matrix;
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
//        String experimentName = "C3DTLZ1_OMOPSO_pop35_iter2000";
//        String problemName =  "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1";
//        String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf";
        String experimentName = "C3DTLZ4_OMOPSO_pop35_iter2000";
        String problemName =  "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4";
        String referenceParetoFront = "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf";

        DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);
        int generations = 2000;
        int iterations = 2;    // 20

        // empty the result folder
        try {
            FileUtils.cleanDirectory(new File(resultFolder));
        }catch(IOException e){
            e.printStackTrace();
        }
/*
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
        // Todo: (1)すべての試行で得られたすべての結果の最大値と最小値で正規化してHVを求めるように修正する
        // Todo: (2)DTLZ1で4目的以上の場合HVが算出できない問題を解決する
        // Todo: (3)OMOPSOだとC3DTLZ4は100個体1000世代では全然うまく探索できていない．他の手法も同様か試す

        // calculate average HyperVolume of each generations
        Matrix hypervolumes = new Matrix(iterations, generations+1);
        Matrix normalizedHypervolumes = new Matrix(iterations, generations+1);
        String hvFolder = archiveFolderBase + experimentName+"\\";

        for( int i=0; i<iterations; i++) {
            for (int g=1; g<=generations; g++) {
                // read fitness in each generation
                String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i) + "\\";
                String solutionsFile = "epsilonFitness"+g+".csv";
                Matrix solutions = new Matrix(Csv.read(archiveFolder + solutionsFile));
                List<DoubleSolution> population = new ArrayList<DoubleSolution>(solutions.length());
                for (int s = 0; s < solutions.length(); s++) {
                    DoubleSolution newIndividual = problem.createSolution();
                    for (int o = 0; o < solutions.columnLength(); o++) {
                        newIndividual.setObjective(o, solutions.get(s, o));
                    }
                    population.add(newIndividual);
                }

                // calculate hypervolume
                try {
                    Front referenceFront = new ArrayFront(referenceParetoFront);
                    FrontNormalizer frontNormalizer = new FrontNormalizer(referenceFront);
                    Front normalizedReferenceFront = frontNormalizer.normalize(referenceFront);
                    Front normalizedFront = frontNormalizer.normalize(new ArrayFront(population));
                    List<PointSolution> normalizedPopulation = FrontUtils.convertFrontToSolutionList(normalizedFront);
                    double normalizedHypervolume = new PISAHypervolume<PointSolution>(normalizedReferenceFront).evaluate(normalizedPopulation);
                    double hypervolume = new PISAHypervolume<DoubleSolution>(referenceFront).evaluate(population);
                    normalizedHypervolumes.set(i, g, normalizedHypervolume);
                    hypervolumes.set(i, g, hypervolume);
                } catch (FileNotFoundException e) {
                    e.printStackTrace();
                }
            }
        }

        // save hypervolume
        Csv.write(hvFolder+"HV(n).csv",normalizedHypervolumes.get());
        Csv.write(hvFolder+"HV.csv",hypervolumes.get());
    }
}
