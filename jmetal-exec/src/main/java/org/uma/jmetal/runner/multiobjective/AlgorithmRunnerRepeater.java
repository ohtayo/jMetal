package org.uma.jmetal.runner.multiobjective;

import org.apache.commons.io.FileUtils;
import org.uma.jmetal.util.JMetalException;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.lang.reflect.Method;
import java.util.regex.Pattern;

/**
 * Class for running algorithms many times.
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
public class AlgorithmRunnerRepeater {
    public static void main(String[] args) throws JMetalException, IOException, FileNotFoundException {
        // folders
        String resultFolder = "C:\\workspace\\jMetal\\result\\";
        String archiveFolderBase = "C:\\workspace\\jMetal\\archive\\";

        // problem and search settings
        String[] problems = {
//            "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ1",
            "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4"
        };
        String[] referenceParetoFronts = {
//            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf",
            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf"
        };
        String[] algorithms = {
//            "ParallelOMOPSO",
//            "ParallelNSGAIIWithEpsilonArchive"
//            "ParallelNSGAIIIWithEpsilonArchive",
//            "ParallelConstraintMOEADWithEpsilonArchive"
//            "ParallelNSGAII",
//            "ParallelNSGAIII",
//            "ParallelConstraintMOEAD",
            "ParallelOMOPSOWithSizeLimitedArchive"
        };
        int numberOfIndividuals = 35;
        int numberOfGenerations = 1000;  //2000
        int numberOfRepeats = 20;    // 20
        int numberOfThreads = 6;

        // run each problem and algorithms
        for (int problemNumber=0; problemNumber<problems.length; problemNumber++){
            // define problem
            String problemName = problems[problemNumber];
            String referenceParetoFront = referenceParetoFronts[problemNumber];
            String[] tempProblemName = problemName.split(Pattern.quote("."));

            // arguments
            args = new String[5];
            args[0] = problemName;
            args[1] = String.valueOf(numberOfIndividuals);
            args[2] = String.valueOf(numberOfGenerations);
            args[3] = String.valueOf(numberOfThreads);
            args[4] = referenceParetoFront;

            for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
                // make experiment name
                String algorithmName = algorithms[algorithmNumber];
                String experimentName = tempProblemName[tempProblemName.length - 1] + "_"+algorithmName+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;

                // run algorithm "numberOfRepeats" times
                for( int i=0; i<numberOfRepeats; i++) {
                    // empty the result folder
                    FileUtils.cleanDirectory(new File(resultFolder));

                    // run algorithm
                    try {
                        Method runnerMain = Class.forName("org.uma.jmetal.runner.multiobjective."+algorithmName + "Runner").getDeclaredMethod("main",new Class[]{ String[].class });
                        runnerMain.invoke(null, new Object[]{ args });
                    } catch (Exception e) {
                        e.printStackTrace();
                    }

                    // copy result
                    String archiveFolder = archiveFolderBase + experimentName + "\\" + String.valueOf(i)+"\\";
                    FileUtils.copyDirectory(new File(resultFolder), new File(archiveFolder));
                }
            }
        }
    }
}
