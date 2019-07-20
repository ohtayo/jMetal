package org.uma.jmetal.util.fileoutput;

import org.uma.jmetal.solution.Solution;
import org.uma.jmetal.util.JMetalException;
import org.uma.jmetal.util.fileoutput.impl.DefaultFileOutputContext;
import org.uma.jmetal.util.solutionattribute.impl.NumberOfViolatedConstraints;
import org.uma.jmetal.util.solutionattribute.impl.OverallConstraintViolation;

import java.io.BufferedWriter;
import java.io.IOException;
import java.util.List;
import java.util.Objects;

/**
 * File output class for constraint violation values.
 * @author ohtayo <ohta.yoshihiro@outlook.jp>
 * @param <S> solution
 */
public class ConstraintListOutput<S extends Solution<?>> {
  private FileOutputContext conFileContext;
  private String conFileName = "CON";
  private String separator = ",";
  private List<S> solutionList;
  public OverallConstraintViolation<S> overallConstraintViolation;
  public NumberOfViolatedConstraints<S> numberOfViolatedConstraints;

  public ConstraintListOutput(List<S> solutionList) {
    conFileContext = new DefaultFileOutputContext(conFileName);
    conFileContext.setSeparator(separator);
    this.solutionList = solutionList;
    overallConstraintViolation = new OverallConstraintViolation<S>() ; // 制約違反の総量
    numberOfViolatedConstraints = new NumberOfViolatedConstraints<S>() ; // 制約違反数
  }

  public ConstraintListOutput setConFileOutputContext(FileOutputContext fileContext) {
    conFileContext = fileContext;
    return this;
  }

  public ConstraintListOutput setSeparator(String separator) {
    this.separator = separator;
    conFileContext.setSeparator(this.separator);
    return this;
  }

  public void print() {
    printConstraintsToFile(conFileContext, solutionList);
  }

  public void printConstraintsToFile(FileOutputContext context, List<S> solutionList) {
    if( Objects.nonNull( numberOfViolatedConstraints.getAttribute(solutionList.get(0)) ) ) {  //制約があれば
      BufferedWriter bufferedWriter = context.getFileWriter();
      try {
        if (solutionList.size() > 0) {  //解が1個以上で
          for (int i = 0; i < solutionList.size(); i++) {
            double temp1 = numberOfViolatedConstraints.getAttribute(solutionList.get(i));
            double temp2 = overallConstraintViolation.getAttribute(solutionList.get(i));
            bufferedWriter.write(temp1 + context.getSeparator());
            bufferedWriter.write(temp2 + context.getSeparator());
            bufferedWriter.newLine();
          }
        }
        bufferedWriter.close();
      } catch (IOException e) {
        throw new JMetalException("Error writing constraints data ", e) ;
      }
    }
  }

  /*
   * Wrappers for printing with default configuration
   */
  public void printConstraintsToFile(String fileName) throws IOException {
    printConstraintsToFile(new DefaultFileOutputContext(fileName), solutionList);
  }

}
