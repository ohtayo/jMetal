package org.uma.jmetal.util.evaluator.impl;

import org.uma.jmetal.problem.Problem;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.evaluator.SolutionListEvaluator;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadPoolExecutor;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.RejectedExecutionException;
import java.util.concurrent.TimeUnit;

/**
 * Evaluator class using ThreadPoolExecutor.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 */
@SuppressWarnings("serial")
public class ThreadPoolSolutionListEvaluator<S> implements SolutionListEvaluator<S> {

  private int numberOfThreads;
  private ThreadPoolExecutor executor;

  public ThreadPoolSolutionListEvaluator(int numberOfThreads, Problem<S> problem) {
    if (numberOfThreads == 0) {
      this.numberOfThreads = Runtime.getRuntime().availableProcessors();
    } else {
      this.numberOfThreads = numberOfThreads;
    }
    JMetalLogger.logger.info("Number of cores: " + numberOfThreads);
  }

  @Override
  public List<S> evaluate(List<S> solutionList, Problem<S> problem)
  {
    executor = (ThreadPoolExecutor)Executors.newFixedThreadPool(numberOfThreads); //スレッドプールを用意
    executor.setCorePoolSize(numberOfThreads);		//コア数のプールサイズを指定
    executor.setMaximumPoolSize(numberOfThreads);	//最大プールサイズを指定

    List<Future<?>> futureList = new ArrayList<Future<?>>();
    Future<?> future;
    try{
      for(int i=0; i<solutionList.size(); i++)
      {
        JMetalLogger.logger.info("Running evaluation for solution "+i);
        future = executor.submit(new EvaluateOne(solutionList.get(i), problem));
        futureList.add(future);
      }
    } catch (RejectedExecutionException e){
      e.printStackTrace();
      JMetalLogger.logger.info(e.getMessage());
    }finally{
      executor.shutdown();	//新規タスクの受付を終了して残ったタスクを継続する．
      try {
        //指定時間が経過するか，全タスクが終了するまで処理を停止する．
        executor.awaitTermination(solutionList.size(), TimeUnit.HOURS);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    return solutionList;
  }

  public int getNumberOfThreads() {
    return numberOfThreads;
  }

  @Override
  public void shutdown() {
    //This method is an intentionally-blank override.
  }

  /**
   * 粒子群評価をマルチスレッドで処理するためのクラス<br>
   * implements Runnable<br>
   */
  public class EvaluateOne implements Runnable{
    private S solution;
    private Problem<S> problem;

    public EvaluateOne(S solution, Problem<S> problem){
      this.solution = solution;
      this.problem = problem;
    }

    public void run(){
      problem.evaluate(solution);
    }
  }
}
