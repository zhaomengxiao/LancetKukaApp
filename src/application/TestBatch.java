package application;

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
import com.kuka.roboticsAPI.motionModel.Motion;
import com.kuka.roboticsAPI.motionModel.MotionBatch;
import com.kuka.roboticsAPI.motionModel.RobotMotion;

import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

public class TestBatch {
	
	private LBRMed  m_robot_ = null;
	
	private   Frame m_originPos = null;
	
	private   int   m_index = 0;
	private   int   m_per = 20;
	
	private MotionBatch m_batch = null;
	
	public TestBatch(LBRMed rb)
	{
		m_robot_ = rb;
		
		System.out.println("TestBatch ");	
	}
	
	public void init()
	{
		m_originPos = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		
		m_index++;
		Frame pos = m_originPos.copy();
		pos.setX(pos.getX() + m_index*m_per);
		pos.setZ(pos.getZ() + m_index*m_per);
		//m_robot_.getFlange().move(ptp(pos));
		
		m_index++;
		Frame pos2 = m_originPos.copy();
		pos2.setX(pos2.getX() + m_index*m_per);
		pos2.setZ(pos2.getZ() + m_index*m_per);
		
		RobotMotion m1 = ptp(pos);
		RobotMotion m2 = ptp(pos2);
		
		if (m_batch == null)
		{
			m_batch = new MotionBatch(m1, m2).setJointVelocityRel(0.2);
			
		}
		
		m_robot_.move(m_batch);
	}
	
	public void run()
	{
		if (m_batch == null) return;
		
		m_batch.getMotions().add(ptp(m_originPos));
		
		m_robot_.move(m_batch);
	}
}
