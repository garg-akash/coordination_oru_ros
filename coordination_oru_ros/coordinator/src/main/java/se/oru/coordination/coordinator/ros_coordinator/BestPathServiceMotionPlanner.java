package se.oru.coordination.coordinator.ros_coordinator;

import java.util.ArrayList;
import java.io.*;
import java.util.HashMap;
import java.util.List;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.Quaternion;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.parameter.ParameterTree;
//import org.ros.message.Time;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;

import geometry_msgs.Point;
import orunav_msgs.Task;
import orunav_msgs.Path;
import orunav_msgs.RobotTarget;
import orunav_msgs.Shape;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;
import orunav_msgs.GetBestPath;
import orunav_msgs.GetBestPathRequest;
import orunav_msgs.GetBestPathResponse;
//import orunav_msgs.GetTaskFromDB;
//import orunav_msgs.GetTaskFromDBRequest;
//import orunav_msgs.GetTaskFromDBResponse;

public class BestPathServiceMotionPlanner extends AbstractMotionPlanner {
	
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private boolean computing = false;
	private boolean outcome = false;
	private int robotID = -1;

	public BestPathServiceMotionPlanner(int robotID, ConnectedNode node, TrajectoryEnvelopeCoordinatorROS tec) {
		this.node = node;
		this.tec = tec;
		this.robotID = robotID;
	}
	
	private RobotTarget makeRobotTarget(Pose goalPose, final int robotID) {
		
		if (!robotIDstoGoalIDs.keySet().contains(robotID)) robotIDstoGoalIDs.put(robotID, 1);
		else robotIDstoGoalIDs.put(robotID, robotIDstoGoalIDs.get(robotID)+1);
		final int goalID = robotIDstoGoalIDs.get(robotID);
		
		RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
		orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
		geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
		geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
		geometry_msgs.Quaternion quat = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
		point.setX(goalPose.getX());
		point.setY(goalPose.getY());
		point.setZ(0.0);
		Quaternion gQuat = new Quaternion(goalPose.getTheta());
		quat.setW(gQuat.getW());
		quat.setX(gQuat.getX());
		quat.setY(gQuat.getY());
		quat.setZ(gQuat.getZ());
		gpose.setPosition(point);
		gpose.setOrientation(quat);
		ps.setPose(gpose);
		ps.setSteering(0.0);
		rt.setGoal(ps);
		
		Pose currentPose = tec.getRobotReport(robotID).getPose();
		orunav_msgs.PoseSteering psInit = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
		geometry_msgs.Pose ipose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
		geometry_msgs.Point pointInit = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
		geometry_msgs.Quaternion quatInit = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
		pointInit.setX(currentPose.getX());
		pointInit.setY(currentPose.getY());
		pointInit.setZ(0.0);
		Quaternion iQuat = new Quaternion(currentPose.getTheta());
		quatInit.setW(iQuat.getW());
		quatInit.setX(iQuat.getX());
		quatInit.setY(iQuat.getY());
		quatInit.setZ(iQuat.getZ());
		ipose.setPosition(pointInit);
		ipose.setOrientation(quatInit);
		psInit.setPose(ipose);
		psInit.setSteering(0.0);
		rt.setStart(psInit);
		
		rt.setRobotId(robotID);
		rt.setTaskId(goalID);
		rt.setGoalId(goalID);
		
		return rt;
		
	}
	
	@Override
	public boolean doPlanning() {
		//TODO: refuse to do planning if the robot is not idle,
		//		as compute_task service will use current pose of
		//		robot as initial pose. 
		if (!(
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.AT_CRITICAL_POINT) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE.WAITING_FOR_TASK) ||
				tec.getVehicleState(robotID).equals(VEHICLE_STATE._IGNORE_))) {
			System.out.println("Not planning because Robot" + robotID + " is not idle (in state " + tec.getVehicleState(robotID) + ")");
			return false;
		}
		
		if (!computing) {
			
			computing = true;
			
			//Instead of calling computeTask below (the original ROS service that calls the lattice-based motion planner),
			//you could directly "ask your roadmap", that is, call your service getBestPath(this.goal[0], robotID, other paths...)
			//To get the other paths, do as above in AKASH Option 2.
			
			//for loop from 1 to numRobots
			//  if robot is not free, add to vector of robots to consider in Task b
			//call spatial adjustment service with current robot for a and computed vector for b

			int numRobot = 2;
			
			final ArrayList<Task> taskBP = new ArrayList<Task>();
			for(int i = 1; i <= numRobot; i++) {
				if(!(tec.isFree(i)) && i!=robotID) {
					taskBP.add(tec.getCurrentTask(i));
				}
			}
			
			System.out.println("Going to call Get Best Path service for " + robotID);
			System.out.println("Current State " + tec.getVehicleState(robotID));
			System.out.println("BPService Funn");
			ServiceClient<GetBestPathRequest, GetBestPathResponse> serviceClientBP;
			try { serviceClientBP = node.newServiceClient("/robot" + robotID + "/get_best_path", GetBestPath._TYPE); }
			catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
			final GetBestPathRequest requestBP = serviceClientBP.newMessage();
			//requestBP.setA(tec.getCurrentTask(robotID));
			RobotTarget rt = makeRobotTarget(this.goal[0], robotID);
			requestBP.setA(rt);
			requestBP.setB(taskBP);
			
			serviceClientBP.call(requestBP, new ServiceResponseListener<GetBestPathResponse>() {
				
				@Override
				public void onFailure(RemoteException e) {
					System.out.println("FAILED to call Get Best Path service for Robot" + robotID);
					outcome = false;
					computing = false;
				}
	
				@Override
				public void onSuccess(GetBestPathResponse resBP) {
					System.out.println("Successfully called Get Best Path service for Robot" + robotID);
					tec.setCurrentTask(resBP.getC().getTarget().getRobotId(), resBP.getC());
					ParameterTree params = node.getParameterTree();
					if(robotID == 1) {
						params.set("/bp_robot1", 1); //parameters to process time to goal of robot1 if it finds a path 
						params.set("/bp_robot1_begin", node.getCurrentTime().toSeconds());
					}
					if(robotID == 2) {
						params.set("/bp_robot2", 1); //parameters to process time to goal of robot2 if it finds a path 
						params.set("/bp_robot2_begin", node.getCurrentTime().toSeconds());
					}

					System.out.println("Seting break deadlock to false when called robot" + robotID);
					tec.setBreakDeadlocksByReordering(false);
					tec.setBreakDeadlocksByReplanning(false);
					ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
					List<orunav_msgs.PoseSteering> p = resBP.getC().getPath().getPath();
					for (int i = 0; i < p.size(); i++) {
						orunav_msgs.PoseSteering onePS = p.get(i);
						Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
						PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
						path.add(ps);
					}
					pathPS = path.toArray(new PoseSteering[path.size()]);
					outcome = true;
					computing = false;
				}
			});
		
		}
		while (computing) try { Thread.sleep(100); } catch (InterruptedException e) { e.printStackTrace(); }
		return outcome;
	}

}
