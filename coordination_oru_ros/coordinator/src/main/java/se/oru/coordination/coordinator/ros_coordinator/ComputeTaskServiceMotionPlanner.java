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
import orunav_msgs.ComputeTask;
import orunav_msgs.ComputeTaskRequest;
import orunav_msgs.ComputeTaskResponse;
import orunav_msgs.Task;
import orunav_msgs.Path;
import orunav_msgs.RobotTarget;
import orunav_msgs.Shape;
import se.oru.coordination.coordination_oru.motionplanning.AbstractMotionPlanner;
import se.oru.coordination.coordinator.ros_coordinator.TrajectoryEnvelopeTrackerROS.VEHICLE_STATE;
import orunav_msgs.SpatialDeviate;
import orunav_msgs.SpatialDeviateRequest;
import orunav_msgs.SpatialDeviateResponse;
import orunav_msgs.GetBestPath;
import orunav_msgs.GetBestPathRequest;
import orunav_msgs.GetBestPathResponse;
//import orunav_msgs.GetTaskFromDB;
//import orunav_msgs.GetTaskFromDBRequest;
//import orunav_msgs.GetTaskFromDBResponse;
//change1

public class ComputeTaskServiceMotionPlanner extends AbstractMotionPlanner {
	
	private ConnectedNode node = null;
	private HashMap<Integer,Integer> robotIDstoGoalIDs = new HashMap<Integer,Integer>();
	private TrajectoryEnvelopeCoordinatorROS tec = null;
	private boolean computing = false;
	private boolean outcome = false;
	private int robotID = -1;

	public ComputeTaskServiceMotionPlanner(int robotID, ConnectedNode node, TrajectoryEnvelopeCoordinatorROS tec) {
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
		
		
		
//		orunav_msgs.PoseSteering st_ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
//		geometry_msgs.Pose st_gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
//		geometry_msgs.Point st_point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
//		geometry_msgs.Quaternion st_quat = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
//		st_point.setX(goalPose.getX());
//		st_point.setY(goalPose.getY());
//		st_point.setZ(0.0);
//		Quaternion st_gQuat = new Quaternion(goalPose.getTheta());
//		st_quat.setW(st_gQuat.getW());
//		st_quat.setX(st_gQuat.getX());
//		st_quat.setY(st_gQuat.getY());
//		st_quat.setZ(st_gQuat.getZ());
//		st_gpose.setPosition(st_point);
//		st_gpose.setOrientation(st_quat);
//		st_ps.setPose(st_gpose);
//		st_ps.setSteering(0.0);
//		rt.setStart(st_ps);
		return rt;
		
	}
	
	//Input to this function is a goa pose, and robot ID
	//See AKASH Option 2 below
	private void callComputeTaskService(Pose goalPose, final int robotID) {

		if (!computing) {
			computing = true;
	
			ServiceClient<ComputeTaskRequest, ComputeTaskResponse> serviceClient;
			try { serviceClient = node.newServiceClient("/robot" + robotID + "/compute_task", ComputeTask._TYPE); }
			catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
		
			final ComputeTaskRequest request = serviceClient.newMessage();
			request.setStartFromCurrentState(true);
			
			request.setTarget(makeRobotTarget(goalPose, robotID));
			
			for (Geometry obs : this.obstacles) {
				Shape shape = node.getTopicMessageFactory().newFromType(Shape._TYPE);
				for (Coordinate coord : obs.getCoordinates()) {
					Point pnt = node.getTopicMessageFactory().newFromType(Point._TYPE);
					pnt.setX(coord.x);
					pnt.setY(coord.y);
					shape.getPoints().add(pnt);
				}
				//Shape is a polygon (type = 1)
				shape.setType(1);
				request.getExtraObstacles().add(shape);
				System.out.println("Added extra obstacle when planning for Robot" + robotID);
			}
			final ArrayList<ComputeTaskResponse> rsd = new ArrayList<ComputeTaskResponse>();
			
			serviceClient.call(request, new ServiceResponseListener<ComputeTaskResponse>() {
				
				@Override
				public void onFailure(RemoteException arg0) {
					System.out.println("FAILED to call ComputeTask service for Robot" + robotID + " (goalID: " + request.getTarget().getGoalId() + ")");
					outcome = false;
					computing = false;
				}
	
				@Override
				public void onSuccess(ComputeTaskResponse arg0) {
					//AKASH option 2
					System.out.println("Successfully called ComputeTask service for Robot" + robotID + " (goalID: " + request.getTarget().getGoalId() + ")");
					outcome = true;
					rsd.clear();
					rsd.add(arg0);
					
					//change2
//					if(robotID == 2) {
//						if(!tec.isFree(1)) {
//							System.out.println("Going to call Spatial Deviate service for" + robotID);
//							ServiceClient<SpatialDeviateRequest, SpatialDeviateResponse> serviceClientSD;
//							try { serviceClientSD = node.newServiceClient("/robot" + robotID + "/get_spatial_deviation", SpatialDeviate._TYPE); }
//							catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
//						
//							final SpatialDeviateRequest requestSD = serviceClientSD.newMessage();
//							requestSD.setPath1(tec.getCurrentTask(1).getPath());
//							requestSD.setPath2(arg0.getTask().getPath());
//							
//							serviceClientSD.call(requestSD, new ServiceResponseListener<SpatialDeviateResponse>() {
//								
//								@Override
//								public void onFailure(RemoteException e) {
//									System.out.println("FAILED to call SpatialDeviate service for Robot" + robotID);
//								}
//					
//								@Override
//								public void onSuccess(SpatialDeviateResponse resSD) {
//									System.out.println("Successfully called SpatialDeviate service for Robot" + robotID);
//									//arg0.getTask().setPath(resSD.getSptlPath2());
//									rsd.get(0).getTask().setPath(resSD.getSptlPath2());
//								}
//							});
//						}
//						else {
//							System.out.println("RobotID 1 is free...Not going to call Spatial Deviate service for" + robotID);
//						}
//					}
					
					//ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
					//try {
						  //FileInputStream fstream = new FileInputStream("/home/aass/Downloads/path2_opt_fine.txt");
						  // Get the object of DataInputStream
						  //DataInputStream in = new DataInputStream(fstream);
						  //BufferedReader br = new BufferedReader(new InputStreamReader(in));
						  //String strLine;
						  //Read File Line By Line
						  //while ((strLine = br.readLine()) != null)   {
						  // Print the content on the console
						  //System.out.println (strLine);
						  //orunav_msgs.PoseSteering onePS = arg0.getTask().getPath().getPath().get(i);
						  //Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
						  //PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
						  //path.add(ps);
						  //}
						  ////Close the input stream
						  //in.close();
						    //}catch (Exception e){//Catch exception if any
						  //System.err.println("Error: " + e.getMessage());
					//}
					
					//Here we are about to give the newly computed task to the coordinator (tec)...
					
					//Before this is done, we may want change the path in the task by calling your ROS service.
					//To do this, we need the paths of other driving robots - tec.isFree(x) returns true iff x is idle
					//For each non-idle robot y, get its task with tec.getCurrentTask(y), from which you can get the path with Task.getPath()
					//Feed all the other paths, plus the path to modify (arg0.getTask().getPath()) to your service
					//Update the arg0.getTask() after your computation, use arg0.getTask().setPath() ...
					
					//change3
					tec.setCurrentTask(rsd.get(0).getTask().getTarget().getRobotId(), rsd.get(0).getTask());
					ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
					for (int i = 0; i < rsd.get(0).getTask().getPath().getPath().size(); i++) {
						orunav_msgs.PoseSteering onePS = rsd.get(0).getTask().getPath().getPath().get(i);
						Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
						PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
						path.add(ps);
					}
					pathPS = path.toArray(new PoseSteering[path.size()]);
					computing = false;
				}
			});
		}
	}
	
//	private void callGetTaskFromDBService(Pose goalPose, final int robotID) {
//
//		if (!computing) {
//			computing = true;
//	
//			ServiceClient<GetTaskFromDBRequest, GetTaskFromDBResponse> serviceClientDB;
//			try { serviceClientDB = node.newServiceClient("/robot" + robotID + "/get_task_from_db", GetTaskFromDB._TYPE); }
//			catch (ServiceNotFoundException e) { throw new RosRuntimeException(e); }
//	
//			if (!robotIDstoGoalIDs.keySet().contains(robotID)) robotIDstoGoalIDs.put(robotID, 1);
//			else robotIDstoGoalIDs.put(robotID, robotIDstoGoalIDs.get(robotID)+1);
//			final int goalID = robotIDstoGoalIDs.get(robotID);
//	
//			final GetTaskFromDBRequest requestDB = serviceClientDB.newMessage();
//			RobotTarget rt = node.getTopicMessageFactory().newFromType(RobotTarget._TYPE);
//			orunav_msgs.PoseSteering ps = node.getTopicMessageFactory().newFromType(orunav_msgs.PoseSteering._TYPE);
//			geometry_msgs.Pose gpose = node.getTopicMessageFactory().newFromType(geometry_msgs.Pose._TYPE);
//			geometry_msgs.Point point = node.getTopicMessageFactory().newFromType(geometry_msgs.Point._TYPE);
//			geometry_msgs.Quaternion quat = node.getTopicMessageFactory().newFromType(geometry_msgs.Quaternion._TYPE);
//			point.setX(goalPose.getX());
//			point.setY(goalPose.getY());
//			point.setZ(0.0);
//			Quaternion gQuat = new Quaternion(goalPose.getTheta());
//			quat.setW(gQuat.getW());
//			quat.setX(gQuat.getX());
//			quat.setY(gQuat.getY());
//			quat.setZ(gQuat.getZ());
//			gpose.setPosition(point);
//			gpose.setOrientation(quat);
//			ps.setPose(gpose);
//			ps.setSteering(0.0);
//			rt.setGoal(ps);
//			rt.setRobotId(robotID);
//			rt.setTaskId(goalID);
//			rt.setGoalId(goalID);
//			requestDB.setTr(rt);
//			
//			serviceClientDB.call(requestDB, new ServiceResponseListener<GetTaskFromDBResponse>() {
//				
//				@Override
//				public void onFailure(RemoteException arg0) {
//					System.out.println("FAILED to call GetTaskFromDB service for Robot" + robotID + " (goalID: " + goalID + ")");
//					outcome = false;
//					computing = false;
//				}
//	
//				@Override
//				public void onSuccess(GetTaskFromDBResponse arg0) {
//					System.out.println("Successfully called GetTaskFromDB service for Robot" + robotID + " (goalID: " + goalID + ")");
//					outcome = true;
//					tec.setCurrentTask(arg0.getTs().getTarget().getRobotId(), arg0.getTs());
//					ArrayList<PoseSteering> path = new ArrayList<PoseSteering>();
//					for (int i = 0; i < arg0.getTs().getPath().getPath().size(); i++) {
//						orunav_msgs.PoseSteering onePS = arg0.getTs().getPath().getPath().get(i);
//						Quaternion quat = new Quaternion(onePS.getPose().getOrientation().getX(), onePS.getPose().getOrientation().getY(), onePS.getPose().getOrientation().getZ(), onePS.getPose().getOrientation().getW());
//						PoseSteering ps = new PoseSteering(onePS.getPose().getPosition().getX(), onePS.getPose().getPosition().getY(), quat.getTheta(), onePS.getSteering());
//						path.add(ps);
//					}
//					pathPS = path.toArray(new PoseSteering[path.size()]);
//					computing = false;
//				}
//			});
//		}
//	}
	
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
		
		//AKASH option 3
		
		if (!computing) {
			
			computing = true;
			
			//Instead of calling computeTask below (the original ROS service that calls the lattice-based motion planner),
			//you could directly "ask your roadmap", that is, call your service getBestPath(this.goal[0], robotID, other paths...)
			//To get the other paths, do as above in AKASH Option 2.
			
			//for loop from 1 to numRobots
			//  if robot is not free, add to vector of robots to consider in Task b
			//call spatial adjustment service with current robot for a and computed vector for b
			//this.callComputeTaskService(this.goal[0], robotID);
			//change1
			int numRobot = 2;
			//this.callGetTaskFromDBService(this.goal[0], robotID); //where should it be placed?
			
			final ArrayList<Task> taskBP = new ArrayList<Task>();
			for(int i = 1; i <= numRobot; i++) {
				if(!(tec.isFree(i)) && i!=robotID) {
					taskBP.add(tec.getCurrentTask(i));
				}
			}
			
			System.out.println("Going to call Get Best Path service for " + robotID);
			System.out.println("Current State " + tec.getVehicleState(robotID));
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
