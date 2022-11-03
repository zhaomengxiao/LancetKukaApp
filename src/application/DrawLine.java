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
public class DrawLine {
	
	public enum LINE_MODEL
    {
        NONE,
        NORMAL_LINE,
        FORCE_LINE,
        FORCE_LINE_EX;
    }
	
	// 运动是否开始
	private boolean m_bStart = false;
	
	// 运动是否完成
	private boolean m_bDone = false;
	
	private LINE_MODEL  m_nLineModel = LINE_MODEL.NONE;

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
	
	private SmartServoLINSimpleMotion m_servoLineMotion = null;
	
	private Frame m_currentFrame = null;
	
	public DrawLine(LBRMed rb, ObserverManager observerMgr, Param param, LINE_MODEL MD)
	{
		m_robot_ = rb;
		m_observerMgr = observerMgr;
		m_nLineModel = MD;
		
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
				
		if (Math.abs(misX) >= Math.abs(misY) && Math.abs(misX) >= Math.abs(misZ))
		{
			m_aix = "X";
			m_tag = (misX > 0.000001 ? 1 : -1);
			m_aixValue = m_xStart;
		}
		else if (Math.abs(misY) >= Math.abs(misZ) && Math.abs(misY) >= Math.abs(misX))
		{
			m_aix = "Y";
			m_tag = (misY > 0.000001 ? 1 : -1);
			m_aixValue = m_yStart;
		}
		else if (Math.abs(misZ) >= Math.abs(misY) && Math.abs(misZ) >= Math.abs(misX))
		{
			m_aix = "Z";
			m_tag = (misZ > 0.000001 ? 1 : -1);
			m_aixValue = m_zStart;
		}				
		
		System.out.println("init start pos " + m_xStart + "  " + m_yStart + "  " + m_zStart);
		System.out.println("init end   pos " + m_xEnd + "  " + m_yEnd + "  " + m_zEnd);
		System.out.println("init aix " + m_aix + m_tag);
		System.out.println(misX + "  " + misY + "  " + misZ);		
	}
	
	public void BindServoLineMotion(SmartServoLINSimpleMotion motion)
	{
		m_servoLineMotion = motion;
	}
	
	public Frame getCurrentPos()
	{
		return m_currentFrame;
	}
	
	public boolean getIsDone()
	{
		return m_bDone;
	}
	
	public void drawLineNormal()
	{
		Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		Frame targetPos = fr.copy();
		targetPos.setX(m_xEnd);
		targetPos.setY(m_yEnd);
		targetPos.setZ(m_zEnd);
				
		m_robot_.move(lin(targetPos).setJointVelocityRel(0.2));	
		
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
		if (m_nLineModel == LINE_MODEL.NORMAL_LINE)
		{
			while (!this.m_bDone)
			{
				run(1.0);
			}
			return;
		}
		
		System.out.println("StartLisener");
		
		// 这里用法兰的坐标系
		CoordinateAxis ax = CoordinateAxis.Z;
		//if (m_aix.equals("Z")) ax = CoordinateAxis.Z;
		//if (m_aix.equals("X")) ax = CoordinateAxis.X;
		//if (m_aix.equals("Y")) ax = CoordinateAxis.Y;
		
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
	
	// 奇异点检查
	public void test()
	{
		// move to home pos
		JointPosition home = new JointPosition(0, 0, 0, 0, 0, 0, 0);

		m_robot_.setHomePosition(home);
		m_robot_.move(ptpHome().setJointVelocityRel(0.3));
		
		// get cur pos
		Frame curFr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());		
		
		curFr.setZ(curFr.getZ() + 20);
		
		try
		{
			// 奇异点检查成功，该点不可达
			JointPosition pos = m_robot_.getInverseKinematicFromFrameAndRedundancy(curFr);			
		}
		catch(Exception e){
			System.out.println("New pos is error 1");
		}		
		
		try
		{
			m_robot_.getFlange().move(ptp(curFr));
		}
		catch(Exception e){
			System.out.println("New pos is error move error");
		}		
		
		try
		{
			// 该点在最高点下，不是奇异点
			//curFr.setZ(curFr.getZ() - 30);
			//JointPosition pos = m_robot_.getInverseKinematicFromFrameAndRedundancy(curFr);	
			System.out.println("to new pos");
		}
		catch(Exception e){
			System.out.println("New pos is error 3333");
		}
		
		
		// 获取某个轴受力
		//double force = lbr_iiwa_7_R800_1.getExternalForceTorque(_toolAttachedToLBR.getDefaultMotionFrame()).getForce().getX();
		
		System.out.println("test end");
	}
	
	private double GetNewX(double v, String aix, double xStart, double yStart, double zStart, double xEnd, double yEnd, double zEnd)
	{
		if (aix.equals("Y"))
			return (v - yStart)/(yEnd - yStart)*(xEnd - xStart) + xStart;
		else if (aix.equals("Z"))
			return (v - zStart)/(zEnd - zStart)*(xEnd - xStart) + xStart;
		return 0;
	}
	
	private double GetNewY(double v, String aix, double xStart, double yStart, double zStart, double xEnd, double yEnd, double zEnd)
	{
		if (aix.equals("X"))
			return (v - xStart)/(xEnd - xStart)*(yEnd - yStart) + yStart;
		else if (aix.equals("Z"))
			return (v - zStart)/(zEnd - zStart)*(yEnd - yStart) + yStart;
		return 0;
	}
	
	private double GetNewZ(double v, String aix, double xStart, double yStart, double zStart, double xEnd, double yEnd, double zEnd)
	{
		if (aix.equals("X"))
			return (v - xStart)/(xEnd - xStart)*(zEnd - zStart) + zStart;
		else if (aix.equals("Y"))
			return (v - yStart)/(yEnd - yStart)*(zEnd - zStart) + zStart;
		return 0;
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
			m_yResult = GetNewY(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
			m_zResult = GetNewZ(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
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
			m_xResult = GetNewX(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
			m_zResult = GetNewZ(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
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
			m_xResult = GetNewX(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
			m_yResult = GetNewY(m_aixValue, m_aix, m_xStart, m_yStart, m_zStart, m_xEnd, m_yEnd, m_zEnd);
		}
		
		System.out.println("run stepRatio " + stepRatio);
		System.out.println("run result pos " + m_xResult + "," + m_yResult + "," + m_zResult);
		System.out.println("target end " + m_xEnd + "  " + m_yEnd + "  " + m_zEnd);
		
		if (m_bDone)
		{
			System.out.println("run done ok");
		}
		
		// move 
		Frame fr = m_robot_.getCurrentCartesianPosition(m_robot_.getFlange());
		fr.setX(m_xResult);
		fr.setY(m_yResult);
		fr.setZ(m_zResult);
		
		try
		{
			// 奇异点检查
			JointPosition pos = m_robot_.getInverseKinematicFromFrameAndRedundancy(fr);	
			
			if (m_nLineModel != LINE_MODEL.FORCE_LINE_EX)
			{
				m_robot_.move(ptp(fr).setJointVelocityRel(0.2));
			}
			//m_robot_.move(ptp(fr));
		}
		catch(Exception e){
			System.out.println("New pos is error ");
			return null;
		}		
		
		if (m_nLineModel == LINE_MODEL.FORCE_LINE_EX && m_servoLineMotion != null)
		{
			//m_servoLineMotion.run(fr);
		}
		
		m_currentFrame = fr;
		
		return fr;
	}
	
	

}
