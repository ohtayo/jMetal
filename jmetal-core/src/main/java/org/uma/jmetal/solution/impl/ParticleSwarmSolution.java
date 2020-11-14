package org.uma.jmetal.solution.impl;

import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.pseudorandom.JMetalRandom;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Defines an implementation of a double solution for particle swarm optimization.
 *
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ParticleSwarmSolution extends DefaultDoubleSolution {
  public List<Double> speed;

  /** Constructor */
  public ParticleSwarmSolution(DoubleProblem problem) {
    super(problem) ;
    speed = new ArrayList<Double>(problem.getNumberOfVariables());
    initializeSpeed(JMetalRandom.getInstance());
  }

  /** Copy constructor */
  public ParticleSwarmSolution(DefaultDoubleSolution solution) {
    super(solution) ;
    speed = new ArrayList<Double>(problem.getNumberOfVariables());
    initializeSpeed();
  }
  /** Copy constructor */
  public ParticleSwarmSolution(DoubleSolution solution) {
    super((DefaultDoubleSolution) solution) ;
    speed = new ArrayList<Double>(problem.getNumberOfVariables());
    initializeSpeed();
  }

  /** Copy constructor */
  public ParticleSwarmSolution(ParticleSwarmSolution solution) {
    super(solution) ;
    List<Double> speed = solution.getSpeed();
    this.setSpeed(speed);
  }

  private void initializeSpeed(JMetalRandom randomGenerator) {
    for (int i = 0 ; i < problem.getNumberOfVariables(); i++) {
      Double value = randomGenerator.nextDouble(getLowerBound(i), getUpperBound(i));
      speed.add(value);
    }
  }
  private void initializeSpeed() {
    for (int i = 0 ; i < problem.getNumberOfVariables(); i++) {
      speed.add(0.0);
    }
  }

  public Double getSpeed(int index) { return speed.get(index); }
  public List<Double> getSpeed() { return speed; }

  public void setSpeed(int index, Double value) { speed.set(index, value); }
  public void setSpeed(List<Double> values) {
    this.speed = new ArrayList<Double>(problem.getNumberOfVariables());
    for (int i = 0; i < problem.getNumberOfVariables(); i++) {
      this.speed.add(values.get(i));
    }
  }

  public DoubleProblem getProblem() { return problem; }

  public DoubleSolution getDoubleSolution()
  {
    DoubleSolution solution = new DefaultDoubleSolution(problem);
    for( int v=0; v<getNumberOfVariables();  v++ ){
      solution.setVariableValue(v, getVariableValue(v));
    }
    for( int o=0; o<getNumberOfObjectives(); o++ ){
      solution.setObjective(o, getObjective(o));
    }
    OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>();
    NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>();
    overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolationDegree.getAttribute(this));
    numberOfViolatedConstraints.setAttribute(solution, numberOfViolatedConstraints.getAttribute(this));

    return solution;
  }

  @Override
  public ParticleSwarmSolution copy() {
    return new ParticleSwarmSolution(this);
  }

  public boolean equals(ParticleSwarmSolution solution){
    for(int v=0; v<getNumberOfVariables(); v++){
      if(this.getVariableValue(v) != solution.getVariableValue(v)){
        return false;
      }
    }
    return true;
  }
}
