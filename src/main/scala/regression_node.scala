package org.ros.ros_figaro

import org.ros.namespace.GraphName
import org.ros.node.AbstractNodeMain
import org.ros.node.NodeMain
import org.ros.node.ConnectedNode

import org.ros.node.service.ServiceResponseBuilder
import org.ros.node.service.ServiceServer

import org.ros.node.DefaultNodeMainExecutor
import org.ros.node.NodeConfiguration
import org.ros.address.InetAddressFactory
import java.net.URI

import org.apache.commons.logging.Log

class RegressionNode(
  run_regression_callback: (ros_figaro.RunRegressionRequest, ros_figaro.RunRegressionResponse) => Unit)
  extends AbstractNodeMain {
  override def getDefaultNodeName(): GraphName = {
    GraphName.of("regression")
  }

  override def onStart(connectedNode: ConnectedNode) {

    val run_regression_resp_builder: ServiceResponseBuilder[ros_figaro.RunRegressionRequest, ros_figaro.RunRegressionResponse] =
      new ServiceResponseBuilder[ros_figaro.RunRegressionRequest, ros_figaro.RunRegressionResponse]() {
        override def build(
          req: ros_figaro.RunRegressionRequest,
          resp: ros_figaro.RunRegressionResponse) {
          run_regression_callback(req, resp)
        }
      }

    connectedNode.newServiceServer(
      "~run", ros_figaro.RunRegression._TYPE, run_regression_resp_builder)
  }

  def execute() {
    val exec = DefaultNodeMainExecutor.newDefault()
    val config = NodeConfiguration.newPublic(
      InetAddressFactory.newFromHostString(sys.env("ROS_IP")).getHostAddress());
    config.setMasterUri(new URI(sys.env("ROS_MASTER_URI")));
    exec.execute(this, config);
  }
}

