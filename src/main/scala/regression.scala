import com.cra.figaro.language._
import com.cra.figaro.algorithm.sampling._
import com.cra.figaro.library.atomic.continuous._

object Regression {
  def main(args: Array[String]): Unit = {
    // Specifying the prior on model parameters
    val theta0 = Normal(0, 5);
    val theta1 = Normal(0, 5);
    val theta2 = Normal(0, 5);

    // Printing prior mean and variance
    println("Prior Mean")
    println("theta0: " + theta0.mean)
    println("theta1: " + theta1.mean)
    println("theta2: " + theta2.mean)
    println(" ")

    println("Prior Variance")
    println("theta0: " + theta0.variance)
    println("theta1: " + theta1.variance)
    println("theta2: " + theta2.variance)
    println(" ")

    // Specifying the Model, given the parameters.
    class y(x1: Double, x2: Double, t0: AtomicNormal, t1: AtomicNormal, t2: AtomicNormal) {
      val mu = Apply(t0, t1, t2, (d0: Double, d1: Double, d2: Double) => d0 + d1 * x1 + d2 * x2)
      val out = Normal(mu, 0.5)
    }

    // Initializing arrays to hold training data
    var x1_obs: Array[Double] = new Array[Double](5)
    var x2_obs: Array[Double] = new Array[Double](5)
    var y_obs: Array[Double] = new Array[Double](5)

    // Specifying the training data
    x1_obs(0) = -2.3381; x2_obs(0) = -0.355;
    x1_obs(1) = 5.2566; x2_obs(1) = -0.215;
    x1_obs(2) = 1.2564; x2_obs(2) = 2.5802;
    x1_obs(3) = -0.1092; x2_obs(3) = 2.4405;
    x1_obs(4) = 1.2380; x2_obs(4) = 2.4546;

    y_obs(0) = 2.9505;
    y_obs(1) = -4.2942;
    y_obs(2) = 6.6941;
    y_obs(3) = 7.7105;
    y_obs(4) = 6.3987;

    // Intializing array to hold observation objects
    var y_inst: Array[y] = new Array[y](5)
    // Intantiating observation objects for each training datapoint  and specfying the observations
    for (i <- Range(0, y_obs.length)) {
      y_inst(i) = new y(x1_obs(i), x2_obs(i), theta0, theta1, theta2)
      y_inst(i).out.observe(y_obs(i))
    }

    //Setting up the inference
    val imp = Importance(5000, theta0, theta1, theta2)
    imp.start; imp.stop

    // Computing the mean and variance of the posterior distribution
    val post_mu_theta0 = imp.expectation(theta0, (d: Double) => d)
    val post_mu_theta1 = imp.expectation(theta1, (d: Double) => d)
    val post_mu_theta2 = imp.expectation(theta2, (d: Double) => d)

    val post_var_theta0 = imp.expectation(theta0, (d: Double) => Math.pow(d - post_mu_theta0, 2))
    val post_var_theta1 = imp.expectation(theta1, (d: Double) => Math.pow(d - post_mu_theta1, 2))
    val post_var_theta2 = imp.expectation(theta2, (d: Double) => Math.pow(d - post_mu_theta2, 2))

    // Printing posterior mean and variance
    println("Posterior Mean")
    println("theta0: " + post_mu_theta0)
    println("theta1: " + post_mu_theta1)
    println("theta2: " + post_mu_theta2)
    println(" ")

    println("Posterior Variance")
    println("theta0: " + post_var_theta0)
    println("theta1: " + post_var_theta1)
    println("theta2: " + post_var_theta2)

    imp.kill;
  }
}
