package org.uma.jmetal.algorithm.singleobjective.particleswarmoptimization;

import org.uma.jmetal.algorithm.impl.AbstractParticleSwarmOptimization;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.SolutionListUtils;
import org.uma.jmetal.util.archive.impl.BoundedDominanceArchive;
import org.uma.jmetal.util.comparator.DominanceComparator;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;
import org.uma.jmetal.util.fileoutput.ConstraintListOutput;
import org.uma.jmetal.util.fileoutput.SolutionListOutput;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

/**
 * Class implementing the OMOPSO algorithm as a single objective optimization solver.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class SingleOMOPSO extends AbstractParticleSwarmOptimization<DoubleSolution, List<DoubleSolution>> {

  private DoubleProblem problem;

  SolutionListEvaluator<DoubleSolution> evaluator;

  private int swarmSize;
  private int archiveSize;
  private int maxIterations;
  private int currentIteration;

  private DoubleSolution[] localBest;
  private BoundedDominanceArchive<DoubleSolution> leaderArchive;
  private BoundedDominanceArchive<DoubleSolution> epsilonArchive;

  private double[][] speed;

  private Comparator<DoubleSolution> dominanceComparator;

  private UniformMutation uniformMutation;
  private NonUniformMutation nonUniformMutation;

  private JMetalRandom randomGenerator;

  private List<DoubleSolution> initialSwarm;

  /** Constructor */
  public SingleOMOPSO(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator,
                      int swarmSize, int maxIterations, int archiveSize, UniformMutation uniformMutation,
                      NonUniformMutation nonUniformMutation) {
    this.problem = problem ;
    this.evaluator = evaluator ;

    this.swarmSize = swarmSize ;
    this.maxIterations = maxIterations ;
    this.archiveSize = archiveSize ;

    this.uniformMutation = uniformMutation ;
    this.nonUniformMutation = nonUniformMutation ;

    localBest = new DoubleSolution[swarmSize];
    leaderArchive = new BoundedDominanceArchive<DoubleSolution>(archiveSize);
    epsilonArchive = new BoundedDominanceArchive<DoubleSolution>(archiveSize);

    dominanceComparator = new DominanceComparator<DoubleSolution>();

    speed = new double[swarmSize][problem.getNumberOfVariables()];

    randomGenerator = JMetalRandom.getInstance() ;

    this.initialSwarm = null;
  }


  @Override protected void initProgress() {
    currentIteration = 1;
    dump();
  }

  @Override protected void updateProgress() {
    currentIteration += 1;
    dump();
  }

  protected void dump(){
      // dump solution list in the searching
      List<DoubleSolution> epsilon = getResult();
      new SolutionListOutput(epsilon)
              .setVarFileOutputContext(new DefaultFileOutputContext("./result/epsilonPosition" + currentIteration + ".csv"))
              .setFunFileOutputContext(new DefaultFileOutputContext("./result/epsilonFitness" + currentIteration + ".csv"))
              .setSeparator(",")
              .print();
      new ConstraintListOutput<DoubleSolution>(epsilon)
              .setConFileOutputContext(new DefaultFileOutputContext("./result/epsilonConstraint" + currentIteration + ".csv"))
              .setSeparator(",")
              .print();
      List<DoubleSolution> leader = this.leaderArchive.getSolutionList();
      new SolutionListOutput(leader)
              .setVarFileOutputContext(new DefaultFileOutputContext("./result/leaderPosition" + currentIteration + ".csv"))
              .setFunFileOutputContext(new DefaultFileOutputContext("./result/leaderFitness" + currentIteration + ".csv"))
              .setSeparator(",")
              .print();
      new ConstraintListOutput<DoubleSolution>(leader)
              .setConFileOutputContext(new DefaultFileOutputContext("./result/leaderConstraint" + currentIteration + ".csv"))
              .setSeparator(",")
              .print();
  }

  @Override protected boolean isStoppingConditionReached() {
    return currentIteration >= maxIterations;
  }

  /*
   * initial populationを設定するためのメソッドと，createInitialPopulationを変更
   */
  public void setInitialSwarm(List<DoubleSolution> initialSwarm) {
    this.initialSwarm = initialSwarm;
  }
  @Override
  public List<DoubleSolution> createInitialSwarm(){
    List<DoubleSolution> swarm = new ArrayList<>(swarmSize);
    if(this.initialSwarm == null){
      for (int i = 0; i < swarmSize; i++) {
        DoubleSolution newIndividual = problem.createSolution();
        swarm.add(newIndividual);
      }
    }else{
      swarm = this.initialSwarm;
    }
    return swarm;
  }


  @Override
  protected List<DoubleSolution> evaluateSwarm(List<DoubleSolution> swarm) {
    swarm = evaluator.evaluate(swarm, problem);

    return swarm ;
  }



  @Override public List<DoubleSolution> getResult() {
    //return this.leaderArchive.getSolutionList();
      return this.epsilonArchive.getSolutionList();
  }

  @Override
  protected void initializeLeader(List<DoubleSolution> swarm) {
    for (DoubleSolution solution : swarm) {
      DoubleSolution particle = (DoubleSolution) solution.copy();
      if (leaderArchive.add(particle)) {
        epsilonArchive.add((DoubleSolution) particle.copy());
      }
    }
  }

  @Override
  protected void initializeParticlesMemory(List<DoubleSolution> swarm)  {
    for (int i = 0; i < swarm.size(); i++) {
      DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
      localBest[i] = particle;
    }
  }

  @Override
  protected void updateVelocity(List<DoubleSolution> swarm)  {
    double r1, r2, W, C1, C2;
    DoubleSolution bestGlobal;

    //Select a global best in leader archive
    bestGlobal = SolutionListUtils.findBestSolution(leaderArchive.getSolutionList(), dominanceComparator);

    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      DoubleSolution bestParticle = (DoubleSolution) localBest[i];

      //Parameters for velocity equation
      r1 = randomGenerator.nextDouble();
      r2 = randomGenerator.nextDouble();
      C1 = randomGenerator.nextDouble(1.5, 2.0);
      C2 = randomGenerator.nextDouble(1.5, 2.0);
      W = randomGenerator.nextDouble(0.1, 0.5);

      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        //Computing the velocity of this particle
        speed[i][var] = W * speed[i][var] + C1 * r1 * (bestParticle.getVariableValue(var) -
            particle.getVariableValue(var)) +
            C2 * r2 * (bestGlobal.getVariableValue(var) - particle.getVariableValue(var));
      }
    }
  }

  /** Update the position of each particle */
  @Override
  protected void updatePosition(List<DoubleSolution> swarm)  {
    for (int i = 0; i < swarmSize; i++) {
      DoubleSolution particle = swarm.get(i);
      for (int var = 0; var < particle.getNumberOfVariables(); var++) {
        particle.setVariableValue(var, particle.getVariableValue(var) + speed[i][var]);
        if (particle.getVariableValue(var) < problem.getLowerBound(var)) {
          particle.setVariableValue(var, problem.getLowerBound(var));
          speed[i][var] = speed[i][var] * -1.0;
        }
        if (particle.getVariableValue(var) > problem.getUpperBound(var)) {
          particle.setVariableValue(var, problem.getUpperBound(var));
          speed[i][var] = speed[i][var] * -1.0;
        }
      }
    }
  }

  @Override
  protected void updateParticlesMemory(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      int flag = dominanceComparator.compare(swarm.get(i), localBest[i]);
      if (flag != 1) {
        DoubleSolution particle = (DoubleSolution) swarm.get(i).copy();
        localBest[i] = particle;
      }
    }
  }

  @Override protected void initializeVelocity(List<DoubleSolution> swarm) {
    for (int i = 0; i < swarm.size(); i++) {
      for (int j = 0; j < problem.getNumberOfVariables(); j++) {
        speed[i][j] = 0.0;
      }
    }
  }

  /**  Apply a mutation operator to all particles in the swarm (perturbation) */
  @Override
  protected void perturbation(List<DoubleSolution> swarm)  {
    nonUniformMutation.setCurrentIteration(currentIteration);

    for (int i = 0; i < swarm.size(); i++) {
      if (i % 3 == 0) {
        nonUniformMutation.execute(swarm.get(i));
      } else if (i % 3 == 1) {
        uniformMutation.execute(swarm.get(i));
      }
    }
  }

  /**
   * Update leaders method
   * @param swarm List of solutions (swarm)
   */
  @Override protected void updateLeaders(List<DoubleSolution> swarm) {
    for (DoubleSolution solution : swarm) {
      DoubleSolution particle = (DoubleSolution) solution.copy();
      if (leaderArchive.add(particle)) {
        epsilonArchive.add((DoubleSolution) particle.copy());
      }
    }

    leaderArchive.sortByDensityEstimator();
    epsilonArchive.sortByDensityEstimator();
  }

  public void tearDown() {
    evaluator.shutdown();
  }

  @Override public String getName() {
    return "SingleOMOPSO" ;
  }

  @Override public String getDescription() {
    return "Single objective optimized MOPSO" ;
  }

}
