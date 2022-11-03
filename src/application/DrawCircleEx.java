package application;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.common.ThreadUtil;
import com.kuka.med.deviceModel.LBRMed;

/**
 * Implementation of a robot application.
 * <p>
 * The application provides a {@link RoboticsAPITask#initialize()} and a 
 * {@link RoboticsAPITask#run()} method, which will be called successively in 
 * the application lifecycle. The application will terminate automatically after 
 * the {@link RoboticsAPITask#run()} method has finished or after stopping the 
 * task. The {@link RoboticsAPITask#dispose()} method will be called, even if an 
 * exception is thrown during initialization or run. 
 * <p>
 * <b>It is imperative to call <code>super.dispose()</code> when overriding the 
 * {@link RoboticsAPITask#dispose()} method.</b> 
 * 
 * @see UseRoboticsAPIContext
 * @see #initialize()
 * @see #run()
 * @see #dispose()
 */
public class DrawCircleEx  {
	//@Inject
	private LBRMed robot_ = null;
	private ProtocolProcess proc = null;
	private IMotionContainer mc = null;
	
	private int index = 0;	
	private double speed = 0.1;	
	private double defaultBetaRad = 0;	
	private Frame  originPosFrame = null;	
	private int    m_sleepTime = 2000;
	private double []homeJoints = {3.44, -37.82, 13.75, 56.15, 3.13, -15.42, -30.3};
	
	public DrawCircleEx(LBRMed rb)
	{
		robot_ = rb;
	}
	
	public void setProcess(ProtocolProcess p){
		proc = p; 
	}
	public void initialize() {
	
		index = 0;
		
		JointPosition def = new JointPosition(
				Math.toRadians(homeJoints[0]), 
				Math.toRadians(homeJoints[1]), 
				Math.toRadians(homeJoints[2]), 
				Math.toRadians(homeJoints[3]), 
				Math.toRadians(homeJoints[4]), 
				Math.toRadians(homeJoints[5]), 
				Math.toRadians(homeJoints[6]));

		robot_.setHomePosition(def);
		robot_.move(ptpHome().setJointVelocityRel(0.2));
	}	
		
	public ProtocolResult getCurrentPos(boolean bFlange, ProtocolResult ret)
	{
		if (bFlange)
		{
			Frame fr = robot_.getCurrentCartesianPosition(robot_.getFlange());
			
			Param pa = new Param();
			pa.setX(fr.getX());
			pa.setY(fr.getY());
			pa.setZ(fr.getZ());
			pa.setA(fr.getGammaRad());
			pa.setB(fr.getBetaRad());
			pa.setC(fr.getAlphaRad());
			ret.setParam(pa);
		}
		
		return ret;
	}
	
	public ProtocolResult run() {
		
		ProtocolResult ret = new ProtocolResult();
		ret.setResultCode(1);
		
		System.out.println("DrawCircle run");
		
		if (index < 0 || index >= 10) return ret;
		
		System.out.println("DrawCircle run index " + index);
		
		ret.setResultCode(0);
		
		Frame flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
	

		if (index == 0)
		{
			
			JointPosition def = new JointPosition(
					Math.toRadians(homeJoints[0]), 
					Math.toRadians(homeJoints[1]), 
					Math.toRadians(homeJoints[2]), 
					Math.toRadians(homeJoints[3]), 
					Math.toRadians(homeJoints[4]), 
					Math.toRadians(homeJoints[5]), 
					Math.toRadians(homeJoints[6]));
			
			mc = robot_.moveAsync(ptp(def).setJointVelocityRel(speed));			
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}

			Frame fr = robot_.getCurrentCartesianPosition(robot_.getFlange());		
			Frame subFr = new Frame(
					fr, 
					0, 0, 0, 
					Math.toRadians(10), 
					0, 
					0);
			
			mc =  robot_.getFlange().moveAsync(ptp(subFr).setJointVelocityRel(speed));
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}
			
			fr = robot_.getCurrentCartesianPosition(robot_.getFlange());
			defaultBetaRad = fr.getBetaRad();
			
			index++;
			
			if (m_sleepTime > 0)
			{
				ThreadUtil.milliSleep(m_sleepTime);
			}
			
			return getCurrentPos(true, ret);
		}
		double allStep = 5.0;		
		if (index > 0 && index < 5)
		{
		
			double angle = defaultBetaRad - Math.toRadians(5+ -12* allStep/2+ 12*index);// - +
			System.out.println("angle " + defaultBetaRad + " " + angle);
			flangeFrame.setBetaRad(angle);
			
			mc =  robot_.getFlange().moveAsync(ptp(flangeFrame).setJointVelocityRel(speed));
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}
			index++;
			
			if (m_sleepTime > 0)
			{
				ThreadUtil.milliSleep(m_sleepTime);
			}
			
			return getCurrentPos(true, ret);
		}
						
		
		/////////////////
		// 恢复原位
		if (index == 5)
		{
			JointPosition def = new JointPosition(
					Math.toRadians(homeJoints[0]), 
					Math.toRadians(homeJoints[1]), 
					Math.toRadians(homeJoints[2]), 
					Math.toRadians(homeJoints[3]), 
					Math.toRadians(homeJoints[4]), 
					Math.toRadians(homeJoints[5]), 
					Math.toRadians(homeJoints[6]));
			
			mc = robot_.moveAsync(ptp(def).setJointVelocityRel(speed));
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}
			Frame fr = robot_.getCurrentCartesianPosition(robot_.getFlange());
			
			// 到达第一个位置
			originPosFrame = fr.copy();
			fr.setAlphaRad(originPosFrame.getAlphaRad() - Math.toRadians(10));
			mc = robot_.getFlange().moveAsync(ptp(fr).setJointVelocityRel(speed));
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}
			
			index++;
			
			if (m_sleepTime > 0)
			{
				ThreadUtil.milliSleep(m_sleepTime);
			}
			
			return getCurrentPos(true, ret);
		}
		
		if (index > 5 && index < 10)
		{
			Frame fr = originPosFrame.copy();
			fr.setAlphaRad(originPosFrame.getAlphaRad() + Math.toRadians(
					5+ -12* allStep/2+(index - 5)*11)
					);

			mc = robot_.getFlange().moveAsync(ptp(fr).setJointVelocityRel(speed));
		 	while (!mc.isFinished()) {
		 		ProtocolResult retCheck = proc.checkTorque();				
				if (retCheck != null) {
				//	log.info("ret != null  " );			
					return retCheck;
				}		
			}
			index++;
			
			if (m_sleepTime > 0)
			{
				ThreadUtil.milliSleep(m_sleepTime);
			}
			
			return getCurrentPos(true, ret);
		}
				
		return ret;
	}
}