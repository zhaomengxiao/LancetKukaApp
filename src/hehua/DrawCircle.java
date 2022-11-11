package hehua;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
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
public class DrawCircle  {
	//@Inject
	private LBRMed robot_ = null;
	
	private BrakeCondition _brakeCondition = null; 
	
	private int index = 0;
	
	public DrawCircle(LBRMed rb)
	{
		robot_ = rb;
	}
	
	public void initialize() {
		// initialize your application here
		// initialize your application here
		index = 0;
		
		JointPosition jp_01 = new JointPosition(0, Math.toRadians(-120 + 1), 0,
				Math.toRadians(-120 + 1), 0, Math.toRadians(-120 + 1), 0);

		robot_.move(ptp(jp_01).setJointVelocityRel(0.3));

		JointPosition jp_02 = new JointPosition(0, 0, 0, Math.toRadians(90), 0,
				Math.toRadians(45), 0);

		robot_.setHomePosition(jp_02);
		robot_.move(ptpHome().setJointVelocityRel(0.3));
	}
	
	public ProtocolResult DefaultPos()
	{
		JointPosition jp_01 = new JointPosition(0, Math.toRadians(-120 + 1), 0,
				Math.toRadians(-120 + 1), 0, Math.toRadians(-120 + 1), 0);

		robot_.move(ptp(jp_01).setJointVelocityRel(0.3));

		JointPosition jp_02 = new JointPosition(0, 0, 0, Math.toRadians(90), 0,
				Math.toRadians(-90), 0);

		robot_.setHomePosition(jp_02);
		robot_.move(ptpHome().setJointVelocityRel(0.3));
		
		ProtocolResult ret = new ProtocolResult();
		return getCurrentPos(true, ret);
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
		
		if (index < 0 || index > 11) return ret;
		
		System.out.println("DrawCircle run index " + index);
		
		ret.setResultCode(0);		
		
		int addDegree = 15;
		
		Frame flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
				
		if (index == 0)
		{
			flangeFrame.setBetaRad(flangeFrame.getBetaRad() -Math.toRadians(45));
			
			robot_.getFlange().move(ptp(flangeFrame).setJointVelocityRel(0.2));
			index++;
			
			
			return getCurrentPos(true, ret);
		}
		
		if (index >= 1 && index < 5)
		{
			flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
			flangeFrame.setBetaRad(flangeFrame.getBetaRad() + Math.toRadians(addDegree));
			
			robot_.getFlange().move(ptp(flangeFrame).setJointVelocityRel(0.2));
			
			index++;
			return getCurrentPos(true, ret);
		}
		
		if (index == 5)
		{
			robot_.move(ptpHome().setJointVelocityRel(0.3));
			
			index++;
			return getCurrentPos(true, ret);
		}			
				
		
		/////////////////
		if (index == 6)
		{
			flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
			flangeFrame.setGammaRad(flangeFrame.getGammaRad() -Math.toRadians(30));
			robot_.getFlange().move(ptp(flangeFrame).setJointVelocityRel(0.2));
			
			index++;
			return getCurrentPos(true, ret);
		}
		
		if (index >= 7 && index < 11)
		{
			flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
			flangeFrame.setGammaRad(flangeFrame.getGammaRad() +Math.toRadians(addDegree));
			robot_.getFlange().move(ptp(flangeFrame).setJointVelocityRel(0.2));
			
			index++;
			return getCurrentPos(true, ret);
		}
		
		if (index == 11)
		{
			robot_.move(ptpHome().setJointVelocityRel(0.3));
			
			index++;
			return getCurrentPos(true, ret);
		}
		
		return ret;
//		
//		try {
//			Thread.sleep(2000);
//		
//		flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
//		flangeFrame.setGammaRad(flangeFrame.getGammaRad() +Math.toRadians(60));
//	
//		robot_.getFlange().move(ptp(flangeFrame));
//		flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
//		flangeFrame.setGammaRad(flangeFrame.getGammaRad() +Math.toRadians(60));
//
//		
//		robot_.getFlange().move(ptp(flangeFrame));
//		
//		flangeFrame = robot_.getCurrentCartesianPosition(robot_.getFlange());
//		flangeFrame.setGammaRad(flangeFrame.getGammaRad() +Math.toRadians(60));
//		
//		robot_.getFlange().move(ptp(flangeFrame));
//		} catch (InterruptedException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}

	}
}