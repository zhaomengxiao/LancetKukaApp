package hehua;

import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.IAnyEdgeListener;
import com.kuka.roboticsAPI.conditionModel.ICallbackAction;
import com.kuka.roboticsAPI.conditionModel.NotificationType;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.executionModel.IFiredTriggerInfo;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.math.CoordinateAxis;
import com.kuka.roboticsAPI.geometricModel.math.Vector;

import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

public class Handguiding_test1
{
	private LBRMed robot =null;
	
	public Handguiding_test1(LBRMed rb)
	{
		robot=rb;
	}
	public void initialize()
	{
		
	}
	public void run(){
	    //robot.move(ptpHome().setJointVelocityRel(0.25));
	    robot.move(ptp(0.0,0.0,0.0,0.0,0.0,0.0,0.0));
	    robot.move(handGuiding());
	}
}