package org.uma.jmetal.problem.multiobjective.ep;

import org.uma.jmetal.problem.impl.AbstractAtOneTimeEvaluableDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.io.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.List;
import java.util.Properties;
import java.util.concurrent.*;

/**
 * Class representing problem CEC2009_UF12
 */
@SuppressWarnings("serial")
public class ZEBRefModelVarDiff4ObjRegretConPMVAtOneTimeEvaluationByManyPCs extends AbstractAtOneTimeEvaluableDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;

  /**
   * Constructor.
   */
  public ZEBRefModelVarDiff4ObjRegretConPMVAtOneTimeEvaluationByManyPCs() {
    setNumberOfVariables(19);
    setNumberOfObjectives(4);
    setNumberOfConstraints(1);
    setName("ZEBRefModelVarDiff4ObjRegretConPMVAtOneTimeEvaluationByManyPCs") ;

    // set limit of variables
    List<Double> lowerLimit = new ArrayList<>(getNumberOfVariables()) ;
    List<Double> upperLimit = new ArrayList<>(getNumberOfVariables()) ;
    for (int i = 0; i < getNumberOfVariables(); i++) {
      lowerLimit.add(0.0);
      upperLimit.add(1.0);
    }
    setLowerLimit(lowerLimit);
    setUpperLimit(upperLimit);

    // set limit of objectives
    minValue = new ArrayList<>(getNumberOfObjectives());
    maxValue = new ArrayList<>(getNumberOfObjectives());
    // objective1
    minValue.add(0.0);
    maxValue.add(2.0);
    // objective2
    minValue.add(2.0e9);
    maxValue.add(9.0e9);
    // objective3
    minValue.add(0.0);
    maxValue.add(2.0);
    // objective4
    minValue.add(0.0);
    maxValue.add(7.0e9);

    constraintViolation = new double[getNumberOfConstraints()];
    for(int i=0; i< getNumberOfConstraints(); i++){
      constraintViolation[i] = 0.0;
    }
    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ; // 制約違反の総量
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ; // 制約違反数

    allocation = new ArrayList<>();
    readProperties();
  }

  private PC[] PCs;
  private List<Integer> allocation;
  private int totalNumberOfThreads;
  private String workFolder;
  private final String propertiesFile = "manypcs.properties";
  private class PC{
    String IPAddress;
    String user;
    String password;
    int numberOfThreads;
    String baseFolder;
  }

  /**
   * Read PCs properties from file.
   */
  private void readProperties(){
    Properties properties = new Properties();
    try {
      properties.load(new FileInputStream(propertiesFile));
      workFolder = properties.getProperty("workFolder");
      int numberOfPCs = Integer.valueOf(properties.getProperty("numberOfPCs"));
      PCs = new PC[numberOfPCs];
      totalNumberOfThreads=0;
      for(int pc=0; pc<PCs.length; pc++){
        PCs[pc] = new PC();
        PCs[pc].IPAddress = properties.getProperty("IPAddress"+pc);
        PCs[pc].user = properties.getProperty("user"+pc);
        PCs[pc].password = properties.getProperty("password"+pc);
        PCs[pc].baseFolder = properties.getProperty("baseFolder"+pc);
        PCs[pc].numberOfThreads = Integer.valueOf(properties.getProperty("numberOfThreads"+pc));
        totalNumberOfThreads+=PCs[pc].numberOfThreads;
        for(int i=0; i<PCs[pc].numberOfThreads; i++) allocation.add(pc);
      }
    } catch (FileNotFoundException e){
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /** Evaluate() method */
  @Override
  public void evaluate(DoubleSolution solution) {
    return;
  }
  @Override
  public void evaluate(List<DoubleSolution> solutionList) {

    // 各実行スレッドを作成する．
    ThreadPoolExecutor executor;
    executor = (ThreadPoolExecutor) Executors.newFixedThreadPool(totalNumberOfThreads); //スレッドプールを用意
    executor.setCorePoolSize(totalNumberOfThreads);		//コア数のプールサイズを指定
    executor.setMaximumPoolSize(totalNumberOfThreads);	//最大プールサイズを指定

    List<Future<?>> futureList = new ArrayList<Future<?>>();
    Future<?> future;
    try{
      for(int i=0; i<solutionList.size(); i++)
      {
        JMetalLogger.logger.info("Running evaluation for solution "+i);
        future = executor.submit(new ZEBRefModelVarDiff4ObjRegretConPMVAtOneTimeEvaluationByManyPCs.EvaluateOne(solutionList.get(i), PCs[allocation.get(i%totalNumberOfThreads)]));
        futureList.add(future);
        sleep(1000);
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

  }

  /**
   * 粒子群評価をマルチスレッドで処理するためのクラス<br>
   * implements Runnable<br>
   */
  public class EvaluateOne implements Runnable{
    private DoubleSolution solution;
    private PC pc;

    public EvaluateOne(DoubleSolution solution, PC pc){
      this.solution = solution;
      this.pc = pc;
    }

    public void run(){
      sleep(1000);
      // スレッド名の取得
      String threadName = Thread.currentThread().getName().split("thread")[1];

      // 変数を文字列に変換
      String variableString = "";
      for(Double variable : solution.getVariables()) variableString+=String.format("%.3f,",variable);
      String commandFileName = workFolder + "\\psexec" + threadName + ".cmd";
      String outFileName = workFolder + "\\psexec" + threadName + ".out";
      // psexecのコマンドを作成
      String command = "cd " + workFolder + " &" +
          " psexec64 -s" +
          " -u " + pc.user +
          " -p " + pc.password +
          " \\\\" + pc.IPAddress +
          " cmd /c" +
          " \"" +
            "cd "+ pc.baseFolder + " & jmetal.cmd" +
            " \"" + threadName + "\"" +
            " \"" + variableString + "\"" +
          "\"" +
          " > " + outFileName;
      System.out.println(command);

      // コマンドをpsexec_スレッド名.cmdで保存
      try{
        Path path = Paths.get(commandFileName);
        if (!Files.exists(path)) Files.createFile(path);  // ファイルがなければ作成
        List<String> lines = new ArrayList<>();
        lines.add(command);
        Files.write(path, lines, StandardOpenOption.WRITE);
      }catch(IOException e){
        e.printStackTrace();
      }

      // コマンドを管理者権限で実行
      String execute = "cmd /c \"powershell start-process " + commandFileName + " -Wait -verb runas & timeout /t 1 > nul\"";
      System.out.println(execute);
      int ret = 0;
      Runtime runtime = Runtime.getRuntime();
      try{
        Process process = runtime.exec(execute);
        ret = process.waitFor();
        System.out.println(ret);
        if(ret!=0)	JMetalLogger.logger.severe("EnergyPlus occurred error(s).");
      }catch(Exception e){
        e.printStackTrace();
      }

      // 出力を読み込み
      double[] f = new double[getNumberOfObjectives()];
      double overallConstraintViolation = 0.0;
      int violatedConstraints = 0;
      try{
        Path path = Paths.get(outFileName);
        List<String> lines = Files.readAllLines(path);
        System.out.println( lines.get(lines.size()-1) );
        String[] result = lines.get(lines.size()-1).split(",");
        for(int i=0; i<f.length; i++) f[i] = Double.valueOf(result[i]);
        overallConstraintViolation = Double.valueOf(result[getNumberOfObjectives()]);
        violatedConstraints = Integer.valueOf(result[getNumberOfObjectives()+1]);
      }catch(IOException e){
        e.printStackTrace();
      }

      // 出力をSolutionに反映
      for (int o = 0; o < getNumberOfObjectives(); o++) solution.setObjective(o, f[o]);
      overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolation);
      numberOfViolatedConstraints.setAttribute(solution, violatedConstraints);
    }
  }

  //time / 1000 秒待ち
  private void sleep(long time){
    try {
      Thread.sleep(time);
    }catch(InterruptedException e){
      e.printStackTrace();
    }
  }
  /**
   * InputStreamを行ごとのListとして返す
   * @param is InputStream
   * @return List
   */
  private List<String> inputStreamToStringArray(InputStream is){
    BufferedReader br = new BufferedReader(new InputStreamReader(is));
    List<String> lines = new ArrayList<>();
    String line;
    try{
      while ( (line = br.readLine()) != null ) {
        lines.add(line);
      }
      br.close();
    }catch(IOException e) {
      e.printStackTrace();
    }
    return lines;
  }
}
