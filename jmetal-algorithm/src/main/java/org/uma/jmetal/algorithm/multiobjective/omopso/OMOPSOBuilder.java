package org.uma.jmetal.algorithm.multiobjective.omopso;

import org.uma.jmetal.operator.MutationOperator;
import org.uma.jmetal.operator.impl.mutation.NonUniformMutation;
import org.uma.jmetal.operator.impl.mutation.UniformMutation;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.AlgorithmBuilder;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

/** Class implementing the OMOPSO algorithm */
public class OMOPSOBuilder implements AlgorithmBuilder<OMOPSO> {
  public enum OMOPSOVariant {OMOPSO, OMOPSOWithSizeLimitedArchive}

  protected DoubleProblem problem;
  protected SolutionListEvaluator<DoubleSolution> evaluator;

  private int swarmSize = 100 ;
  private int archiveSize = 100 ;
  private int maxIterations = 25000 ;
  private double eta = 0.0075;

  private UniformMutation uniformMutation ;
  private NonUniformMutation nonUniformMutation ;

  private OMOPSOVariant variant;

  public OMOPSOBuilder(DoubleProblem problem, SolutionListEvaluator<DoubleSolution> evaluator) {
    this.evaluator = evaluator ;
    this.problem = problem ;
    this.variant = OMOPSOVariant.OMOPSO;
  }

  public OMOPSOBuilder setSwarmSize(int swarmSize) {
    this.swarmSize = swarmSize ;

    return this ;
  }

  public OMOPSOBuilder setArchiveSize(int archiveSize) {
    this.archiveSize = archiveSize ;

    return this ;
  }

  public OMOPSOBuilder setMaxIterations(int maxIterations) {
    this.maxIterations = maxIterations ;

    return this ;
  }

  public OMOPSOBuilder setEta(double eta) {
    this.eta = eta ;

    return this ;
  }

  public OMOPSOBuilder setUniformMutation(MutationOperator<DoubleSolution> uniformMutation) {
    this.uniformMutation = (UniformMutation)uniformMutation ;

    return this ;
  }

  public OMOPSOBuilder setNonUniformMutation(MutationOperator<DoubleSolution> nonUniformMutation) {
    this.nonUniformMutation = (NonUniformMutation)nonUniformMutation ;

    return this ;
  }

  public OMOPSOBuilder setVariant(OMOPSOVariant variant){
    this.variant = variant;
    return this;
  }

  /* Getters */
  public int getArchiveSize() {
    return archiveSize;
  }

  public int getSwarmSize() {
    return swarmSize;
  }

  public int getMaxIterations() {
    return maxIterations;
  }

  public double getEta() {
    return eta;
  }

  public UniformMutation getUniformMutation() {
    return uniformMutation;
  }

  public NonUniformMutation getNonUniformMutation() {
    return nonUniformMutation;
  }

  public SolutionListEvaluator<DoubleSolution> getSolutionListEvaluator() { return evaluator; }

  public OMOPSO build() {
    OMOPSO algorithm = null;
    if(this.variant == OMOPSOVariant.OMOPSO){
      algorithm = new OMOPSO(problem, evaluator, swarmSize, maxIterations, archiveSize, uniformMutation,
          nonUniformMutation, eta);
    }else if(this.variant == OMOPSOVariant.OMOPSOWithSizeLimitedArchive){
      algorithm = new OMOPSOWithSizeLimitedArchive(problem, evaluator, swarmSize, maxIterations, archiveSize, uniformMutation,
          nonUniformMutation, eta);
    }
    return algorithm ;
  }
}
