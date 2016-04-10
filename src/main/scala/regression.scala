package org.ros.ros_figaro

import com.cra.figaro.language._
import com.cra.figaro.algorithm.sampling._
import com.cra.figaro.library.atomic.continuous._

import org.ros.node.NodeConfiguration

object Regression {

  val node = new RegressionNode(run_regression_callback)
  node.execute()

  class y(x: List[Double], w: AtomicMultivariateNormal, v: Double = 5.0 ) {
    def regress(d: List[Double]): Double = {
      var res = d(0)
      for (i <- 0 to x.length - 1) {
          res += d(i + 1) * x(i)
      }
      res
    }
    val mu = Apply(w, regress)
    val out = Normal(mu, v)
  }

  def run_regression_callback(
    req: ros_figaro.RunRegressionRequest,
    resp: ros_figaro.RunRegressionResponse) {

    Universe.createNew()

    // Create MultivariateNormal from the request
    val n = req.getPrior().getMean().length
    var cov = scala.collection.mutable.ListBuffer.empty[List[Double]]
    for(i <- 0 to n - 1){
        cov += req.getPrior().getCovar().slice(i * n, (i + 1) * n).toList
    }
    val w = MultivariateNormal(req.getPrior().getMean().toList, cov.toList)

    // Add the last observation
    val data_point = new y(req.getObservation().getX().toList, w)
    data_point.out.observe(req.getObservation().getY())

    // Set up and run inference
    val imp = Importance(5000, w)
    imp.start

    // Get the posterior mean
    var mean =  new Array[Double](n)
    for(i <- 0 to n - 1)  {
      mean(i) = imp.expectation(w, (d: List[Double]) => d(i))
    }
    resp.getPosterior().setMean(mean)

    // Get the posterior covariance
    var covar = new Array[Double](n * n)

    // First fill in the diagonal
    for(i <- 0 to n - 1) {
      covar(i * n + i) = imp.expectation(w,
          (d: List[Double]) => (d(i) - mean(i)) * (d(i) - mean(i)))
    }

    // Then fill in the rest taking into account the symmetry
    for(i <- 1 to n - 1) {
      for(j <- 0 to i - 1) {
        covar(i * n + j) = imp.expectation(
          w, (d: List[Double]) => (d(i) - mean(i)) * (d(j) - mean(j)))
        covar(j * n + i) = covar(i * n + j)
      }
    }
    resp.getPosterior().setCovar(covar)

    // (Double, Map[Element[_], Any])
    val nsamples = 100
    val samples = new Array[imp.Sample](nsamples)

    for (i <- Range(0, samples.length)) {
       samples(i) = imp.sample()
    }

    resp.setSamples(new java.util.ArrayList[ros_figaro.ImportanceSample])
    for (i <- Range(0, samples.length)) {
      val s: ros_figaro.ImportanceSample =
        NodeConfiguration.newPrivate().getTopicMessageFactory().newFromType(
          ros_figaro.ImportanceSample._TYPE);

      s.setWeight(samples(i)._1)
      s.setValue(samples(i)._2(w).asInstanceOf[w.Value].toArray)
      resp.getSamples().add(s)
    }

    imp.kill
  }

  def main(args: Array[String]): Unit = {
  }
}
