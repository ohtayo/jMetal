package org.uma.jmetal.runner.multiobjective;

import org.apache.commons.io.FileUtils;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.ProblemUtils;

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
//            "org.uma.jmetal.problem.multiobjective.cdtlz.C3_DTLZ4",
//            "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_S_DTLZ2",
//            "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_S_DTLZ3",
//            "org.uma.jmetal.problem.multiobjective.cec2007MOAlgorithmCompetition.C3_R_DTLZ2",
//            "org.uma.jmetal.problem.multiobjective.newDtlz.C3_RosenbrockDTLZ2",
//            "org.uma.jmetal.problem.multiobjective.UF.C3_UF11",
//            "org.uma.jmetal.problem.multiobjective.UF.C3_UF12",
//            "org.uma.jmetal.problem.multiobjective.UF.C3_UF12MatlabEngineAtOneTimeEvaluation",
//            "org.uma.jmetal.problem.multiobjective.maf.C3_MaF02",
//            "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff2ObjConPMV",
//            "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelLSTMVarDiff2ObjConPMVUDP",
            "org.uma.jmetal.problem.multiobjective.ep.ZEBRefModelVarDiff2ObjConPMV",
        };
        String[] referenceParetoFronts = {
//            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.4D.pf",
//            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ4.4D.pf",
//            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ3.5D.pf",
//            "jmetal-problem/src/test/resources/pareto_fronts/UF12.pf",
//            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ2.3D.pf",
            "jmetal-problem/src/test/resources/pareto_fronts/DTLZ1.2D.pf",
        };
        String[] algorithms = {
//                "ParallelNSGAII",
//                "ParallelNSGAIIWithEpsilonArchive",
//                "ParallelNSGAIII",
//                "ParallelNSGAIIIWithEpsilonArchive",
//                "ParallelConstraintMOEAD",
//                "ParallelConstraintMOEADDE",
//                "ParallelConstraintMOEADWithEpsilonArchive",
//                "ParallelConstraintMOEADDEWithEpsilonArchive",
                "ParallelOMOPSO",
//                "ParallelOMOPSOWithSizeLimitedArchive",
//                "ParallelOMOPSORV",
                "ParallelOMOPSODBT",
//                "ParallelOMOPSODBT2",
//                "ParallelOMOPSODBT3",
//                "ParallelOMOPSODBT4",
//                "ParallelOMOPSODBT5",
//                "ParallelOMOPSORVDBT",
//                "ParallelOMOPSORVDBT2",
//                "ParallelOMOPSORVAOP",
//                "ParallelOMOPSORVIBP",
//                "ParallelOMOPSORVPPS",
//                "ParallelOMOPSODBTDFG",
//                "ParallelOMOPSODBTIBG",
//                "ParallelOMOPSORVDBTDFG",
//                "ParallelOMOPSORVDBTIBG",
//                "ParallelOMOPSOCSS",
//                "ParallelOMOPSONDX",
//                "ParallelOMOPSOPPS",
//                "ParallelDirectionalOMOPSOWithSizeLimitedArchive",
                "ParallelOMOPSODegradeLeader",
                "ParallelOMOPSODegradeMutation",
                "ParallelOMOPSODegradeArchiveSize",
                "ParallelOMOPSODegradeSelection",
        };
        // file names of initial solutions
        String[] initialSolutions = {
                "",
        };

        int numberOfIndividuals = 35;
        int numberOfGenerations = 500;  //2000
        int numberOfRepeats = 1;    // 20
        int numberOfThreads = 6;

        // run each problem and algorithms
        for (int problemNumber=0; problemNumber<problems.length; problemNumber++){
            // define problem
            String problemName = problems[problemNumber];
            String referenceParetoFront = referenceParetoFronts[problemNumber];
            String initialSolution = initialSolutions[problemNumber];

            // check population size for MOEA/D
            DoubleProblem problem = (DoubleProblem) ProblemUtils.<DoubleSolution> loadProblem(problemName);
            if( problem.getNumberOfObjectives()==3 || problem.getNumberOfObjectives()==8 ) {
                numberOfIndividuals = 36;
            }else{
                numberOfIndividuals = 35;
            }

            for(int algorithmNumber=0; algorithmNumber<algorithms.length; algorithmNumber++) {
                // make experiment name
                String algorithmName = algorithms[algorithmNumber];
                String experimentName = problem.getName() + "_"+algorithmName+"_pop" + numberOfIndividuals + "_gen" + numberOfGenerations;

                // make arguments
                if( algorithms[algorithmNumber].contains("ParallelOMOPSO") ) {
                    algorithmName = "ParallelOMOPSO";
                    args = new String[7];
                    args[6] = algorithms[algorithmNumber].replace("Parallel","");
                }else{
                    args = new String[6];
                }
                args[0] = problemName;
                args[1] = String.valueOf(numberOfIndividuals);
                args[2] = String.valueOf(numberOfGenerations);
                args[3] = String.valueOf(numberOfThreads);
                args[4] = referenceParetoFront;
                args[5] = initialSolution;

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
