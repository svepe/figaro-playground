name := "ros-figaro"

version := "0.0.0"

scalaVersion := "2.11.8"
libraryDependencies  ++= Seq(
  "com.cra.figaro" %% "figaro" % "latest.release",
  "org.ros.rosjava_core" % "rosjava" % "latest.release",
  "org.ros.rosjava_messages" % "ros_figaro" %  "latest.release"
)

resolvers ++= Seq(
  // Online 
  "rosjava online" at "https://raw.githubusercontent.com/rosjava/rosjava_mvn_repo/master",
  // Generated rosjava messes
  "rosjava msgs" at "file://" + baseDirectory.value + "/../../devel/share/maven/"
)

// Useful for debugging. Call with `sbt hello`
// TaskKey[Unit]("hello") := println(baseDirectory.value)
