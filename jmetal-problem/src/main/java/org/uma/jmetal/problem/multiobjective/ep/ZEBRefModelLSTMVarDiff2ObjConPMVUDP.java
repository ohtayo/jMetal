package org.uma.jmetal.problem.multiobjective.ep;

import jp.ohtayo.building.energyplus.EnergyPlusObjectives;
import jp.ohtayo.commons.io.Csv;
import jp.ohtayo.commons.log.Logging;
import jp.ohtayo.commons.math.Matrix;
import org.uma.jmetal.problem.DoubleProblem;
import org.uma.jmetal.problem.impl.AbstractDoubleProblem;
import org.uma.jmetal.solution.DoubleSolution;
import org.uma.jmetal.util.JMetalLogger;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Properties;

/**
 * Multi-objective Problem of air-conditioning setpoint temperature schedule optimization using EnergyPlus building energy simulator.
 * Building model: reference building model in the ZEB design guideline
 * Variable : setpoint temperature schedule between 5:00 and 24:00 using difference (length:20)
 * Objective 1: power consumption
 * Objective 2: thermal comfort level
 * Constraint 1: exceedance of comfort level
 *
 * @author ohtayo (ohta.yoshihiro@outlook.jp)
 */
@SuppressWarnings("serial")
public class ZEBRefModelLSTMVarDiff2ObjConPMVUDP extends AbstractDoubleProblem {
  public OverallConstraintViolation<DoubleSolution> overallConstraintViolationDegree ;
  public NumberOfViolatedConstraints<DoubleSolution> numberOfViolatedConstraints ;
  public double[] constraintViolation ;
  public List<Double> minValue;
  public List<Double> maxValue;

  private final String propertiesFile = "lstm.properties";
  private Integer receivePort;
  private Integer sendPort;
  private String address;
  /**
   * Constructor.
   */
  public ZEBRefModelLSTMVarDiff2ObjConPMVUDP() {

    setNumberOfVariables(19);
    setNumberOfObjectives(2);
    setNumberOfConstraints(1);
    setName("ZEBRefModelLSTMVarDiff2ObjConPMVUDP") ;

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
    minValue.add(1.0e9);
    maxValue.add(9.0e9);

    constraintViolation = new double[getNumberOfConstraints()];
    for(int i=0; i< getNumberOfConstraints(); i++){
      constraintViolation[i] = 0.0;
    }
    overallConstraintViolationDegree = new OverallConstraintViolation<DoubleSolution>() ; // 制約違反の総量
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<DoubleSolution>() ; // 制約違反数

    Properties properties = new Properties();
    try {
      properties.load(new FileInputStream(propertiesFile));
      receivePort = Integer.valueOf(properties.getProperty("receivePort"));
      sendPort = Integer.valueOf(properties.getProperty("sendPort"));
      address = properties.getProperty("address");
    } catch (FileNotFoundException e){
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void evaluate(DoubleSolution solution)  {
    // 設定温度スケジュールデータ作成
    List<Double> variablesList = solution.getVariables();
    double[] variables = new double[variablesList.size()];
    for (int i = 0; i < variablesList.size(); i++)  variables[i] = variablesList.get(i);
    double[] temperature = EnergyPlusObjectives.variableToTemperatureSettingUsingDifference(variables, 25, 6,25);
    String sendData = "";
    String receivedData = "";
    for(double temp : temperature) sendData+= temp +",";
    try {
      // 受信の準備
      DatagramSocket socketReceive = new DatagramSocket(receivePort);//UDP受信用ソケット構築
      byte[] dataReceive = new byte[1024];//受信最大バッファ
      DatagramPacket packetReceive = new DatagramPacket(dataReceive, dataReceive.length);//受信用パケットを構築

      // 設定温度スケジュールの送信
      byte[] dataSend = sendData.getBytes(StandardCharsets.UTF_8);//UTF-8バイト配列の作成
      DatagramSocket socketSend = new DatagramSocket();//UDP送信用ソケットの構築
      DatagramPacket packetSend = new DatagramPacket(dataSend, dataSend.length, new InetSocketAddress(address, sendPort));//指定アドレス、ポートへ送信するパケットを構築
      socketSend.send(packetSend);//パケットの送信
      //System.out.println("UDP送信:" + sendData);  // for debug

      // 目的関数値と制約値の受信
      socketReceive.receive(packetReceive);
      receivedData = new String(Arrays.copyOf(packetReceive.getData(),packetReceive.getLength()), StandardCharsets.UTF_8);
      System.out.println("UDP受信:"+receivedData);//受信データの表示
      socketReceive.close();//ソケットのクローズ
      socketSend.close();//ソケットのクローズ
    }catch(IOException e){
      e.printStackTrace();
    }

    double[] fitness = new double[getNumberOfObjectives()];
    double[] constraints = new double[getNumberOfConstraints()];
    String[] result = receivedData.split(",");
    fitness[0] = Double.valueOf(result[0]);  // 対象時刻の中間階PMVを取得して平均の絶対値とる
    fitness[1] = Double.valueOf(result[1]); // 3列目(消費電力)データを取得して合計値を取る．6回に1回間引かれているので6倍する．
    constraints[0] = Double.valueOf(result[2]); //対象時刻の中間階PMVが±0.5をはみ出ている回数を積算する

    // Normalize objective values
    double[] normalizedFitness = new double[getNumberOfObjectives()];
    for(int o=0; o<getNumberOfObjectives(); o++) {
      normalizedFitness[o] = (fitness[o] - minValue.get(o)) / (maxValue.get(o) - minValue.get(o));
    }

    // 評価値と制約違反量を格納
    for(int o=0; o<getNumberOfObjectives(); o++) {
      solution.setObjective(o, normalizedFitness[o]);
    }
    constraintViolation = constraints;
    this.evaluateConstraints(solution);

    // for debug 制約がAttributeに反映されない場合があるので，反映されていなければ無理やり値を入れる
    OverallConstraintViolation<DoubleSolution> overallConstraintViolation;
    overallConstraintViolation = new OverallConstraintViolation<DoubleSolution>() ;
    double violation = overallConstraintViolation.getAttribute(solution);
    double comfort = solution.getObjectives()[0]*2;
    double power = solution.getObjectives()[1];
    if( (violation==0) && ((comfort>0.5)||(power<0.25)) ){
      JMetalLogger.logger.severe("illegal objective values.");
      overallConstraintViolationDegree.setAttribute(solution, 100.0);
      numberOfViolatedConstraints.setAttribute(solution, 1);
    }
  }

  /** EvaluateConstraints() method */
  private void evaluateConstraints(DoubleSolution solution)  {

    double overallConstraintViolation = 0.0;
    int violatedConstraints = 0;
    for (int i = 0; i < getNumberOfConstraints(); i++) {
      if ( (-1 * constraintViolation[i]) <0.0){
        overallConstraintViolation-=constraintViolation[i]; //制約違反量に負の値を与える
        violatedConstraints++;
      }
    }

    overallConstraintViolationDegree.setAttribute(solution, overallConstraintViolation);
    numberOfViolatedConstraints.setAttribute(solution, violatedConstraints);
  }
}
