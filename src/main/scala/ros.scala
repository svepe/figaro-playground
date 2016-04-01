//package org.ros.rosscala_tutorial_pubsub

import org.ros.message.MessageListener
import org.ros.node.AbstractNodeMain
import org.ros.namespace.GraphName
import org.ros.node.ConnectedNode
import org.ros.node.NodeMain
import org.ros.node.topic.Subscriber
import org.ros.node.DefaultNodeMainExecutor
import org.ros.node.NodeConfiguration
import org.ros.address.InetAddressFactory
import java.net.URI

import org.apache.commons.logging.Log

class Listener extends AbstractNodeMain {

  override def getDefaultNodeName(): GraphName = {
    GraphName.of("rosjava_tutorial_pubsub/listener")
  }

  override def onStart(connectedNode: ConnectedNode) {
    var log = connectedNode.getLog();
    var subscriber: Subscriber[ros_figaro.Normal2] = connectedNode.newSubscriber(
      "chatter", ros_figaro.Normal2._TYPE);

    subscriber.addMessageListener(new MessageListener[ros_figaro.Normal2]() {
      override def onNewMessage(message: ros_figaro.Normal2) {
        log.info("I heard: \"" + message.getMean() + "\"");
      }
    });
  }
}

object ListenerProgram {
  def main(args: Array[String]) {
    val l = new Listener()
    val exec = DefaultNodeMainExecutor.newDefault()
    val config = NodeConfiguration.newPublic(
      InetAddressFactory.newFromHostString("192.168.106.24").getHostAddress());
    config.setMasterUri(new URI(sys.env("ROS_MASTER_URI")));
    exec.execute(l, config);
  }
}
