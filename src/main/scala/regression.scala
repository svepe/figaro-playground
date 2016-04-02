package org.ros.ros_figaro

import com.cra.figaro.language._
import com.cra.figaro.algorithm.sampling._
import com.cra.figaro.library.atomic.continuous._

import org.ros.node.NodeConfiguration

object Regression {

  val node = new RegressionNode(run_regression_callback)
  node.execute()

  def run_regression_callback(
    req: ros_figaro.RunRegression2Request,
    resp: ros_figaro.RunRegression2Response) {

    Universe.createNew()

    val w0 = Normal(req.getPriorW0().getMean(), req.getPriorW0().getVariance());
    val w1 = Normal(req.getPriorW1().getMean(), req.getPriorW1().getVariance());

    // Specifying the Model, given the parameters.
    class y(x1: Double, t0: AtomicNormal, t1: AtomicNormal) {
      val mu = Apply(t0, t1, (d0: Double, d1: Double) => d0 + d1 * x1)
      val out = Normal(mu, 5)
    }

    val data_point = new y(req.getObservation().getX(), w0, w1)
    data_point.out.observe(req.getObservation().getY())

    //Setting up the inference
    val imp = Importance(5000, w0, w1)
    imp.start

    resp.getPosteriorW0().setMean(imp.mean(w0))
    resp.getPosteriorW1().setMean(imp.mean(w1))
    resp.getPosteriorW0().setVariance(imp.variance(w0))
    resp.getPosteriorW1().setVariance(imp.variance(w1))

    // (Double, Map[Element[_], Any])
    val nsamples = 10
    val samples = new Array[imp.Sample](nsamples)
    // Intantiating observation objects for each training datapoint  and specfying the observations
    for (i <- Range(0, samples.length)) {
      samples(i) = imp.sample()
    }

    resp.setSamplesW0(new java.util.ArrayList[ros_figaro.ImportanceSample])
    resp.setSamplesW1(new java.util.ArrayList[ros_figaro.ImportanceSample])
    for (i <- Range(0, samples.length)) {

      val s: ros_figaro.ImportanceSample = NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(ros_figaro.ImportanceSample._TYPE);
      s.setWeight(samples(i)._1)
      s.setValue(samples(i)._2(w0).toDouble)
      resp.getSamplesW0().add(s)

      s.setWeight(samples(i)._1)
      s.setValue(samples(i)._2(w1).toDouble)
      resp.getSamplesW0().add(s)

      println("weight: " + math.exp(samples(i)._1) + " w0: " + samples(i)._2(w0) + " w0: " + samples(i)._2(w0))
    }

    imp.kill
  }

  def main(args: Array[String]): Unit = {
  }
}
