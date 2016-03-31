name := "ros-figaro"

version := "0.0.0"

scalaVersion := "2.11.8"
libraryDependencies  ++= Seq(
  "com.cra.figaro" %% "figaro" % "latest.release",
  "org.scalanlp" %% "breeze" % "latest.release",
  "org.scalanlp" %% "breeze-natives" % "latest.release",
  "org.scalanlp" %% "breeze-viz" % "latest.release",
  "org.ros.rosjava_core" % "rosjava" % "latest.release"
)

resolvers ++= Seq(
  "rosjava" at "https://raw.githubusercontent.com/rosjava/rosjava_mvn_repo/master"
)
