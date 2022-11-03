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

import com.kuka.roboticsAPI.conditionModel.ConditionObserver;
import com.kuka.roboticsAPI.deviceModel.JointPosition;

public class TestDrag {
	
	private LBRMed  m_robot_ = null;
	
	private ObserverManager m_observerMgr = null;	
	
	private ConditionObserver m_observer = null;
	
	private double            m_times = 0;
	
	public TestDrag(LBRMed rb, ObserverManager observerMgr)
	{
		m_robot_ = rb;
		
		System.out.println("TestDrag " + (observerMgr == null));
		m_observerMgr = observerMgr;
		
	}
			
	// 开始力条件监听
	public void StartLisener()
	{
		System.out.println("StartLisener");
		
		CoordinateAxis ax = CoordinateAxis.Z;
				
		Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		
		ForceCondition fc = ForceCondition.createNormalForceCondition(m_robot_.getFlange(), 
				fr, ax, 15.0, 6);
		
		//ForceCondition fc = ForceCondition.createNormalForceCondition(
			//	null, m_robot_.getFlange(), ax, 20.0, 6);
		
		IAnyEdgeListener listener = new IAnyEdgeListener(){
			@Override
			public void onAnyEdge(ConditionObserver conditionObserver, Date time, int MissedEvents,
					boolean conditionValue) {
				// TODO Auto-generated method stub
				System.out.println("onAnyEdge " + MissedEvents + "  " + conditionValue + "    " + m_times);
				
				m_times++;
				
				
			}			
		};
		
		if (m_observer != null)
		{
			m_observer.disable();
			m_observerMgr.removeConditionObserver(m_observer);
		}
		
		m_observer = m_observerMgr.createConditionObserver(fc, NotificationType.EdgesOnly, listener);
		
		System.out.println("StartLisener m_observer " + m_observer.isEnabled());
		
		m_observer.enable();
		
		System.out.println("StartLisener m_observer " + m_observer.isEnabled());
		
		System.out.println("StartLisener end");
	}
	
	public void StopListener()
	{
		if (m_observer != null)
		{
			System.out.println("StopListener ");
			m_observer.disable();
		}
	}

}
