package application;

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

/**
 * split many pos on line
 */
public class DragLineOnY {
			
	// 运动是否开始
	private boolean m_bStart = false;
	
	// 运动是否完成
	private boolean m_bDone = false;
	
	
	private LBRMed  m_robot_ = null;
	
	private ObserverManager m_observerMgr = null;
	
	//开始结束点坐标	
	private double  m_xStart = 0.0;
	private double  m_yStart = 0.0;
	private double  m_zStart = 0.0;
	
	private double  m_xEnd = 0.0;
	private double  m_yEnd = 0.0;
	private double  m_zEnd = 0.0;
	
	// 运动轴-世界坐标系
	private String  m_aix = null;
	
	// 当前轴上值
	private double  m_aixValue = 0.0;
	
	// 步长
	private double  m_stepping = 1;	
	
	// 坐标变化方向
	private int     m_tag = 1;
	
	// 监听事件执行次数
	private double  m_times = 1;
	
	// 默认受力大小
	private double  m_defaultForce = 6.0;
	
	private ConditionObserver m_observer = null;
	
	private Frame m_currentFrame = null;
	
	public DragLineOnY(LBRMed rb, ObserverManager observerMgr, Param param)
	{
		m_robot_ = rb;
		m_observerMgr = observerMgr;
		
		// GET CURRENT POS
		Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		
		m_xStart = fr.getX();
		m_yStart = fr.getY();
		m_zStart = fr.getZ();
		
		m_xEnd = param.getX();
		m_yEnd = param.getY();
		m_zEnd = param.getZ();
		
		double misX = m_xEnd - m_xStart;
		double misY = m_yEnd - m_yStart;
		double misZ = m_zEnd - m_zStart;
				
		m_aix = "Y";
		if (m_aix.equals("X"))
		{
			m_aixValue = m_xStart;
			m_tag = (misX > 0 ? 1 : -1);
		}
		else if (m_aix.equals("Y"))
		{
			m_aixValue = m_yStart;
			m_tag = (misY > 0 ? 1 : -1);
		}
		else if (m_aix.equals("Z"))
		{
			m_aixValue = m_zStart;
			m_tag = (misZ > 0 ? 1 : -1);
		}		
		
		System.out.println("init start pos " + m_xStart + "  " + m_yStart + "  " + m_zStart);
		System.out.println("init end   pos " + m_xEnd + "  " + m_yEnd + "  " + m_zEnd);
		System.out.println("init aix " + m_aix + m_tag);
		System.out.println(misX + "  " + misY + "  " + misZ);		
	}
		
	public Frame getCurrentPos()
	{
		return m_currentFrame;
	}
	
	public double getCurrentAixValue()
	{
		return m_aixValue;
	}
	
	public boolean getIsDone()
	{
		return m_bDone;
	}	
		
	public boolean isStart() {
		return m_bStart;
	}

	public void setStart(boolean bStart) {
		this.m_bStart = bStart;
	}
	
	public double getStepping() {
		return m_stepping;
	}

	public void setStepping(double stepping) {
		this.m_stepping = stepping;
	}	
		
	// 开始力条件监听
	public void StartLisener()
	{		
		System.out.println("StartLisener");
		
		// 这里用法兰的坐标系
		CoordinateAxis ax = CoordinateAxis.Z;
		
		Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		
		m_currentFrame = fr;
		
		ForceCondition fc = ForceCondition.createNormalForceCondition(m_robot_.getFlange(), 
				fr, ax, 15.0, 6);		
				
		IAnyEdgeListener listener = new IAnyEdgeListener(){
			@Override
			public void onAnyEdge(ConditionObserver conditionObserver, Date time, int MissedEvents,	boolean conditionValue) 
			{
				// TODO Auto-generated method stub	
				if (conditionValue == false) return;
				
				if (m_bDone)
				{
					System.out.println("DragLine onAnyEdge Done");
					return;
				}
				
				System.out.println("DragLine onAnyEdge " + MissedEvents + "  " + conditionValue + "   " + m_times);
				
				m_times++;
				
				Vector vecF = m_robot_.getExternalForceTorque(m_robot_.getFlange()).getForce();
				//double stepRatio = (vecF.getX() < -m_defaultForce ? 1.0 : Math.abs(vecF.getX()/m_defaultForce));
				//run(stepRatio);
				run(1.0);
				
				/*while (true)
				{
					Vector vec = m_robot_.getExternalForceTorque(m_robot_.getFlange()).getForce();
					System.out.println("after run force is " + vec.getX() + " " + vec.getY() + " " + vec.getZ());
					if (vec.getX() < -m_defaultForce && m_bDone == false)
					{
						System.out.println("run with if condition");
						stepRatio = Math.abs(vec.getX()/m_defaultForce);
						run(stepRatio);
					}
					else
					{
						break;
					}
				}*/
				
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
		
		System.out.println("StartLisener end m_observer " + m_observer.isEnabled());
		
	}
	
	public void StopListener()
	{
		if (m_observer != null)
		{
			System.out.println("StopListener ");
			m_observer.disable();
		}
	}
		
	public Frame run(double stepRatio)
	{
		if (m_bDone) 
		{
			System.out.println("run done");
			return null;
		}
		
		double  m_xResult = 0.0;
		double  m_yResult = 0.0;
		double  m_zResult = 0.0;
		
		double stepping = m_stepping*stepRatio;
		
		if (m_aix.equals("X"))
		{
			if (m_tag == 1 && m_aixValue < m_xEnd)
			{
				if (m_aixValue + stepping >= m_xEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue + stepping < m_xEnd ? m_aixValue + stepping : m_xEnd);			
			}
			else if (m_tag == -1 && m_aixValue > m_xEnd)
			{
				if (m_aixValue - stepping <= m_xEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue - stepping > m_xEnd ? m_aixValue - stepping : m_xEnd);
			}
			
			m_xResult = m_aixValue;
		}
		else if (m_aix.equals("Y"))
		{
			if (m_tag == 1 && m_aixValue < m_yEnd)
			{
				if (m_aixValue + stepping >= m_yEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue + stepping < m_yEnd ? m_aixValue + stepping : m_yEnd);			
			}
			else if (m_tag == -1 && m_aixValue > m_yEnd)
			{
				if (m_aixValue - stepping <= m_yEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue - stepping > m_yEnd ? m_aixValue - stepping : m_yEnd);
			}	
			
			m_yResult = m_aixValue;
		}
		else if (m_aix.equals("Z"))
		{
			if (m_tag == 1 && m_aixValue < m_zEnd)
			{
				if (m_aixValue + stepping >= m_zEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue + stepping < m_zEnd ? m_aixValue + stepping : m_zEnd);			
			}
			else if (m_tag == -1 && m_aixValue > m_zEnd)
			{
				if (m_aixValue - stepping <= m_zEnd) m_bDone = true;
				
				m_aixValue = (m_aixValue - stepping > m_zEnd ? m_aixValue - stepping : m_zEnd);
			}
			
			m_zResult = m_aixValue;
		}
		
		System.out.println("run stepRatio " + stepRatio);
		System.out.println("run result pos " + m_xResult + "," + m_yResult + "," + m_zResult);
		System.out.println("target end " + m_xEnd + "  " + m_yEnd + "  " + m_zEnd);
		
		if (m_bDone)
		{
			System.out.println("run done ok");
		}
		
		// move 
		/*Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		fr.setX(m_xResult);
		fr.setY(m_yResult);
		fr.setZ(m_zResult);
		
		try
		{
			// 奇异点检查
			JointPosition pos = m_robot_.getInverseKinematicFromFrameAndRedundancy(fr);				
		}
		catch(Exception e){
			System.out.println("New pos is error ");
			return null;
		}
						
		m_currentFrame = fr;*/
		
		return null;
	}
	
	

}
