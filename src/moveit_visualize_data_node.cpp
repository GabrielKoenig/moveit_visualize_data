/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Gabriel Koenig, University of Basel, gabriel.koenig@unibas.ch */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/remote_control.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// REMARK: This node should run properly with the adapted moveit branch on my repository on github (melodic_devel_vis_states).
/// I adapted moveit, to be able to store the motion plan request data (edges and vertices) and visualize them in rviz through a separate node
/// Within RVIZ the MarkerArray needs to subscribe to the /rviz_visual_tools topic. The red dots represents the states
/// sampled visulized the specified link (LINK_NAME_VIS), the red lines specify the edges.
/// Code information:
/// - I added the storing capabilities in model_based-Planning_context.cpp within the postSolve() function
/// - The function to visualize the data is declared in the ompl_planner_manager.cpp:
/// void visualizeSampledStates(const planning_scene::PlanningSceneConstPtr& planning_scene,
///                            const std::string& PLANNING_GROUP,
///                            const std::string& LINK_NAME,
///                            const std::string& REFERENCE_FRAME,
///                            const std::string& state_space_model,
///                            const char *filename_to_load)  override
/// IMPORTANT: the OMPL data is only properly stored if no parallel plan is used, ony if the PlanningAttempts=1 is set, you can
/// visualize the data
/// This node can be launched with "roslaunch moveit_visualize_data sampling_data_analysis.launch" and everytime "next"
/// is pressed in the RvizVisualToolsGui the last stored planner data is visualized.
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_visualize_data_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle nh("~"); // private namespace to load the plugin lib

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///  SETUP
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // specify with respect to wich link you want to visualize the sampled data
  static const std::string LINK_NAME_VIS="panda_link8";
  // specify the reference frame of the marker (header.frame_id)
  std::string REFERENCE_FRAME="world";
  // specify the Planning Group, that had been used to generate the samples
  std::string PLANNING_GROUP_FOR_ANALYSIS="panda_arm";
  // specify the state space factory type (can be trial and error or just print out the factory type that had been used during the last motion plan request)
  std::string STATE_SPACE_MODEL="JointModel";

  // this is the default path to the data stored (within ompl/base/PlannerDataStorrage.cpp, when storing the edges and vertices after a motion plan request)
  // the stored path can be changed on the ompl side, or also when calling the storing function in the model_based-planning_context.cpp file)
  const char *path_samples, *path_solution;
  path_samples= "sampled_states";


  // Instantiating a RobotModelLoader object, which will look up the robot description on the ROS Paramter server and
  // construct a RobotModel to use
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group
   *The JointModelGroup represents the robot model for a particular group*/
  robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
  const robot_state::JointModelGroup* joint_model_group_panda = robot_state->getJointModelGroup(PLANNING_GROUP_FOR_ANALYSIS);

  // Using the :moveit_core:`RobotModel`, we can construct a :planning_scene:`PlanningScene`
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));


  ///////////////////////////////////////////////////////////////
  /// Load Planner Plugin
  // We need to load the planner, to access the ompl interface to visualize the data

  // We will now construct a loader to load a planner, by name.
  // Note that we are using the ROS pluginlib library here.
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // We will get the name of planning plugin we want to load
  // from the ROS parameter server, and then load the planner
  // making sure to catch all exceptions.
  if (!nh.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, nh.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (std::size_t i = 0; i < classes.size(); ++i)
      ss << classes[i] << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

    
  ///////////////////////////////////////////////////////
  /// Initialize visualization tools

  /* The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
   and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script*/
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(LINK_NAME_VIS);
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // clear all old markers
  visual_tools.trigger();

  /* Remote control is an introspection tool that allows users to step through a high level script
     via buttons and keyboard shortcuts in RViz */
  visual_tools.loadRemoteControl();
  rviz_visual_tools::RemoteControlPtr remote_ctrl=visual_tools.getRemoteControl();

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///  ANALYSIS OF SAMPLED STATES
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  
  while(!remote_ctrl->getStop()) {
    
    // use the new function implemented within the ompl_planner_manager.cpp (ompl interface), to visualize the sampled states (which were stored in a file)
    planner_instance->visualizeSampledStates(planning_scene, PLANNING_GROUP_FOR_ANALYSIS,LINK_NAME_VIS, REFERENCE_FRAME, STATE_SPACE_MODEL, path_samples);
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to visualize the next samples and delete the current markers");
    visual_tools.deleteAllMarkers();
    sleep(5);
  }
  
  ros::shutdown();


  return 0;
}
