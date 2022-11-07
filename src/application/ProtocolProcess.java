package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;

import  com.kuka.roboticsAPI.deviceModel.*;
import  com.kuka.roboticsAPI.motionModel.*;
import  com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import  com.kuka.roboticsAPI.motionModel.controlModeModel.*;
import com.kuka.med.mastering.Mastering;
import com.kuka.med.jogging.Jogging;

//import java.awt.Frame;
import java.io.IOException;
import java.net.ServerSocket;
import java.util.ArrayList;
import java.util.EnumSet; 
import java.util.Timer;
import java.util.concurrent.TimeUnit;

import application.DrawLine.LINE_MODEL;

import com.kuka.common.ThreadUtil;
import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.conditionModel.IORangeCondition;
import com.kuka.roboticsAPI.conditionModel.JointTorqueCondition;
import com.kuka.roboticsAPI.conditionModel.ObserverManager;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.CommandInvalidException;
import com.kuka.roboticsAPI.geometricModel.*;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.redundancy.IRedundancyCollection;
import com.kuka.task.ITaskLogger;

import functions.*;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

public class ProtocolProcess {

	private DrawCircleEx  m_drawCircle = null;
	private DrawLine    m_drawLine = null;
	private DragLineOnY    m_dragLineOnY = null;
	private SmartServoLINSimpleMotion m_servoLineMotion = null;
	private TestDrag    m_testDrag = null;
	private TestBatch   m_testBatch = null;
	private SmartServoLINNormal   m_sslNormal = null;
	private FreeHandMotion m_freehandMotion = null;
	private IMotionContainer mc = null;
	private PositionControlMode posMode = new PositionControlMode();
	private Frame  frameOrigin = null;
	private LBRMed m_robot = null;
	private Tool tool;
	
	private ObserverManager m_observerMgr = null;
	private BrakeTestHandler m_brakeTestExecutor = null;
	private PositionAndGMS pgms = null;
	private boolean isSoftMode = false;
	private Mastering mastering;
	private Jogging jogging;
    private ITaskLogger log;
    private ArmRobotApp m_app;
    private double speedLevel = 1.0;
    private double a3;
    private double b3;
    private double c3;
    private int i;
    private boolean axis4stop = true;
    private boolean Movestop = false;
	
    
	public ProtocolProcess(LBRMed rb, ITaskLogger logger, Tool t, ObserverManager observerMgr)
	{
		m_robot = rb;
		log = logger;
		m_observerMgr = observerMgr;
		mastering = new Mastering(rb);
		jogging = new Jogging(rb);
		pgms = new PositionAndGMS(rb, logger);
		tool = t;
	}
	
	public void setBrakeTestExecutor(BrakeTestHandler brake)
	{
		m_brakeTestExecutor = brake;
	}
	public void  setApp(ArmRobotApp app) {
		m_app = app;
	}
	public  boolean IsSoftMode()  {
		return isSoftMode; 
	}
	
	public ProtocolResult checkStop() {
		ProtocolBean msg = m_app.peekMsgBean();
		if (msg != null && msg.getOperateType().equals("MoveStop")) {
			mc.cancel();
			m_app.getMsgBean();
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(msg.getOperateType());
			return ret; 
		}
		return null;		
	}
	public ProtocolResult GetJointPos(){
		ProtocolBean msg = m_app.peekMsgBean();
		if(msg != null && msg.getOperateType().equals("GetJointPos")){
			JointPosition jtPos = m_robot.getCurrentJointPosition();
			Frame frm = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
			ProtocolResult ret = new ProtocolResult();	
			Param  prm = new Param();
			prm.x = frm.getX();
			prm.y = frm.getY();
			prm.z = frm.getZ();
			prm.a = Math.toDegrees(frm.getAlphaRad());
			prm.b = Math.toDegrees(frm.getBetaRad());
			prm.c = Math.toDegrees(frm.getGammaRad());
			log.info("WWWWWWWW");
			m_app.getMsgBean();
			ret.setParam(prm);
			ret.setResultMsg(jtPos.toString());
			return ret;
		}
		return null;
		
	}
	
	public ProtocolResult checkTorque() {
		double[] vals = m_robot.getExternalTorque().getTorqueValues();
		for (double value : vals) {
			if (Math.abs(value) > 20) {
				mc.cancel();
				log.info("checkTorque " +
						m_robot.getExternalTorque().toString());
				ProtocolResult ret = new ProtocolResult();
				ret.setOperateType("MotionBlock");
				ret.setResultMsg("MotionBlock ===");
				return ret;
			}
		}
		
		return null;
	}
	public ProtocolResult ProcessData(ProtocolBean bean)
	{
		if (bean == null) {
			log.info("bean null======");
			ProtocolResult result = new ProtocolResult();
			result.setResultCode(0);
			result.setResultMsg("fail");
			return result;
		}
		String opType = bean.getOperateType();
		if (opType.equals("Reset"))
		{
			return Reset(bean);
		}
		else if (opType.equals("StartDrawCircle"))
		{
			return StartDrawCircle(bean);    // å¼€å§‹ç”»åœ†
		}
		else if (opType.equals("DrawCircle"))
		{
			return DrawCircle(bean);         // ç”»åœ†
		}
		else if (opType.equals("StartDragLine"))
		{
			return StartDragLine(bean);    // å¼€å§‹æ‹–åŠ¨æ²¿ç›´çº¿è¿�åŠ¨
		}
		else if (opType.equals("StartDragLineManual"))
		{
			return StartDragLineManual(bean);    // å¼€å§‹æ‹–åŠ¨æ²¿ç›´çº¿è¿�åŠ¨-æ‰‹åŠ¨ä¸€æ­¥ä¸€æ­¥
		}
		else if (opType.equals("ServoMove"))
		{
			return ServoMove(bean);    // å¼€å§‹ä¼ºæœ�è¿�åŠ¨-ä¸–ç•Œå��æ ‡ç³»
		}
		else if (opType.equals("EndDragLine"))
		{
			return EndDragLine(bean);      // ç»“æ�Ÿè¿�åŠ¨
		}
		else if (opType.equals("StartDrawLine"))
		{
			return StartDrawLine(bean);    // å¼€å§‹æ²¿ç›´çº¿è¿�åŠ¨
		}
		else if (opType.equals("StartDrawLineNormal"))
		{
			return StartDrawLineNormal(bean);    // å¼€å§‹æ²¿ç›´çº¿è¿�åŠ¨
		}
		else if (opType.equals("StartLisener"))
		{
			return StartLisener();
		}
		else if (opType.equals("StopListener"))
		{
			return StopListener();
		}
		else if (opType.equals("StartPositionLisener"))
		{
			return StartPositionLisener(bean);
		}
		else if (opType.equals("StopPositionLisener"))
		{
			return StopPositionLisener();
		}
		else if (opType.equals("IsPathViaSingularPoint"))
		{
			IsPathViaSingularPoint();
		}
		else if (opType.equals("IsPathViaSingularPoint2"))
		{
			IsPathViaSingularPoint2();
		}
		else if (opType.equals("Test"))
		{
			Test(bean);
		}
		else if (opType.equals("TestBrake"))
		{
			return TestBrake(bean);
		}
		else if (opType.equals("TestBatch"))
		{
			TestBatch(bean);
		}
		else if (opType.equals("FreeHand"))
		{
			return freeHand();
		}
		else if (opType.equals("MoveStop"))
		{
			mc.cancel();
			m_app.getMsgBean();
			Movestop = true;
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("MoveStop");
			return ret;
		}
		//é›¶ç©ºé—´
		else if (opType.equals("ZeroSpace"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			isSoftMode = true;
			CartesianImpedanceControlMode softMode = new CartesianImpedanceControlMode();
			softMode.parametrize(CartDOF.ALL).setDamping(0.7);
			softMode.parametrize(CartDOF.A).setStiffness(5);
			softMode.parametrize(CartDOF.A).setStiffness(5);
			softMode.parametrize(CartDOF.B).setStiffness(5);
			softMode.parametrize(CartDOF.C).setStiffness(5);
			softMode.parametrize(CartDOF.X).setStiffness(5000);
			softMode.parametrize(CartDOF.Y).setStiffness(5000);
			softMode.parametrize(CartDOF.Z).setStiffness(5000);
			softMode.setMaxPathDeviation(5.0, 5.0, 5.0, 3.14, 3.14, 3.14);
			mc = m_robot.getFlange().moveAsync(positionHold(softMode,-1,TimeUnit.SECONDS));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("ZeroSpace_on");
			ret.setResultMsg("ZeroSpace_on ok");
			return ret;
		}
		//ç¢°æ’žæ£€æµ‹
		else if (opType.equals("Extern_Force"))
		{
			double[] joint = bean.getJointPos();
			JointPosition jp = new JointPosition(
					Math.toRadians(joint[0]),
					Math.toRadians(joint[1]),
					Math.toRadians(joint[2]),
					Math.toRadians(joint[3]),
					Math.toRadians(joint[4]),
					Math.toRadians(joint[5]),
					Math.toRadians(joint[6]));
		    ForceCondition littleforce = ForceCondition.createSpatialForceCondition(m_robot.getFlange(),5);
			ICondition conda ;
			conda = littleforce;
		    mc = m_robot.getFlange().moveAsync(ptp(jp).setJointVelocityRel(0.3* speedLevel)
		    		.setJointAccelerationRel(0.02).breakWhen(conda));
		}
		//é›¶é‡�åŠ›
		else if (opType.equals("Zero_Gravity"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			isSoftMode = true; 
			JointImpedanceControlMode  softMode = new JointImpedanceControlMode(5, 5, 5, 5, 5, 5, 5);
			softMode.setDamping(0.4);
			mc = m_robot.getFlange().moveAsync(positionHold(softMode, -1, TimeUnit.SECONDS));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("SoftMode_On");		
			ret.setResultMsg("SoftMode_On ok");
			ret.setResultCode(0);
			return ret;
		}
		else if (opType.equals("JointPos"))
		{
			double[] joint = bean.getJointPos();
			Movestop = false;
			JointPosition jp = new JointPosition(
					Math.toRadians(joint[0]),
					Math.toRadians(joint[1]),
					Math.toRadians(joint[2]),
					Math.toRadians(joint[3]),
					Math.toRadians(joint[4]),
					Math.toRadians(joint[5]),
					Math.toRadians(joint[6]));
			JointTorqueCondition torqueCondJ1 = new JointTorqueCondition(JointEnum.J1,-25,25);
			JointTorqueCondition torqueCondJ2 = new JointTorqueCondition(JointEnum.J2,-25,25);
			JointTorqueCondition torqueCondJ3 = new JointTorqueCondition(JointEnum.J3,-25,25);
			JointTorqueCondition torqueCondJ4 = new JointTorqueCondition(JointEnum.J4,-25,25);
			JointTorqueCondition torqueCondJ5 = new JointTorqueCondition(JointEnum.J5,-25,25);
			JointTorqueCondition torqueCondJ6 = new JointTorqueCondition(JointEnum.J6,-25,25);
			JointTorqueCondition torqueCondJ7 = new JointTorqueCondition(JointEnum.J7,-25,25);
			//ForceCondition littleforce = ForceCondition.createSpatialForceCondition(m_robot.getFlange(),5);
			ICondition conda ;
			conda = torqueCondJ1.or(torqueCondJ2,torqueCondJ3,torqueCondJ4,
					torqueCondJ5,torqueCondJ6,torqueCondJ7);
		    mc = m_robot.moveAsync(ptp(jp).setJointVelocityRel(0.3* speedLevel)
		    		.setJointAccelerationRel(0.02).breakWhen(conda));
		    //boolean m = m_robot.checkTorqueSensor(JointEnum.J1);
			while (!mc.isFinished()) {
//				ProtocolResult ret = checkStop();				
//				if (ret != null) {
//					log.info("ret_checkstop != null" );		
//					return ret;
//				}
				ProtocolResult ret = GetJointPos();
				if(ret != null){
					log.info("ret_getjointpos != null ");
					return ret;
				}
			   }
			log.info("ret 8888888888= null  " );
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("JointPos");		
			ret.setResultMsg("JointPos ok");
			return ret;
		}	
			else if (opType.equals("FingerMove"))
		{
			double[] joint = bean.getJointPos();
			Param p = bean.getParam();			 
			Transformation offset = Transformation.ofDeg(
					p.x, p.y, p.z, p.a, p.b, p.c);
			log.info("Transformation " + offset.toString());
			IMotion mov = linRel(offset)
					.setJointVelocityRel(0.3 * speedLevel)
					.setJointAccelerationRel(0.02);
			mc = m_robot.getFlange().moveAsync(mov);
			//mc = tool.getFrame("Finger").moveAsync(mov);
		  	while (!mc.isFinished()) {
				ProtocolResult ret = checkStop();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}				
				ret = checkTorque();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}		
			}
			log.info("ret 8888888888= null  " );
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(bean.getOperateType());		
			ret.setResultMsg("FingerMove ok");
			return ret;
		}		
		else if (opType.equals("LineMode_On"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			isSoftMode = true; 
			CartesianImpedanceControlMode  softMode = new CartesianImpedanceControlMode();
			softMode.parametrize(CartDOF.ALL).setDamping(1);
			softMode.parametrize(CartDOF.ROT).setStiffness(300);
			softMode.parametrize(CartDOF.X).setStiffness(5000);
			softMode.parametrize(CartDOF.Y).setStiffness(50);	
			softMode.parametrize(CartDOF.Z).setStiffness(5000);
			
			mc = m_robot.moveAsync(positionHold(softMode, -1, TimeUnit.SECONDS));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("SoftMode_On");		
			ret.setResultMsg("SoftMode_On ok");
			ret.setResultCode(0);
			return ret;
		}
		else if (opType.equals("ConeMode_On"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			CartesianImpedanceControlMode  softMode = new CartesianImpedanceControlMode();
			softMode.parametrize(CartDOF.ALL).setDamping(0.7);
			softMode.parametrize(CartDOF.A).setStiffness(40);
			softMode.parametrize(CartDOF.B).setStiffness(40);
			softMode.parametrize(CartDOF.C).setStiffness(40);
			softMode.parametrize(CartDOF.X).setStiffness(5000);
			softMode.parametrize(CartDOF.Y).setStiffness(5000);	
			softMode.parametrize(CartDOF.Z).setStiffness(5000);
			softMode.setMaxPathDeviation(5.0, 5.0, 5.0, 1.0, 0.26, 0.26);
			mc = tool.getFrame("RobotUserTool").moveAsync(positionHold(softMode, -1, TimeUnit.SECONDS));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("SoftMode_On");		
			ret.setResultMsg("SoftMode_On ok");
			ret.setResultCode(0);
			return ret;
		}
		else if (opType.equals("AutoMode_On"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			Param p =bean.getParam();
			double [] endpoint = {p.x,p.y,p.z,p.a,p.b,p.c};
			mc = tool.getFrame("RobotUserTool").moveAsync(ptp(endpoint)
					.setJointVelocityRel(0.05).setJointAccelerationRel(0.02));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("SoftMode_On");		
			ret.setResultMsg("SoftMode_On ok");
			ret.setResultCode(0);
			return ret;
		}
		else if (opType.equals("SoftMode_On"))
		{
			if (mc != null) {			
				mc.cancel();
			}
			isSoftMode = true; 
			JointImpedanceControlMode  softMode = new JointImpedanceControlMode(5, 5, 5, 5, 5, 5, 5);
			mc = m_robot.moveAsync(positionHold(softMode, -1, TimeUnit.SECONDS));
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("SoftMode_On");		
			ret.setResultMsg("SoftMode_On ok");
			ret.setResultCode(0);
			return ret;
		}
		else if (opType.equals("SoftMode_Off"))
		{
			return SoftModeOff();
		
		}
		else if (opType.equals("Cube"))
		{
//			Frame  frm = m_robot.getCurrentCartesianPosition(
//		 			m_robot.getFlange(), World.Current.getRootFrame());
		//	Transformation tsf = m_robot.getFlange().get();
			
			Param p = bean.getParam();			 
			Transformation offset = Transformation.ofDeg(
					p.x, p.y, p.z, p.a, p.b, p.c);
			log.info("Transformation " + offset.toString());	

			Transformation desTrans = offset.compose(
					frameOrigin.copy().transformationFromWorld());
			Frame desFrame = new Frame(desTrans);

			log.info("frameOrigin" + frameOrigin.toString());	
			log.info("desFrame   " + desFrame.toString());		
			try {
			mc = tool.getDefaultMotionFrame().moveAsync(lin(desFrame)
					.setJointVelocityRel(0.3* speedLevel)
					.setJointAccelerationRel(0.02));
			
			while (!mc.isFinished()) {
				ProtocolResult ret = checkStop();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}				
				ret = checkTorque();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}		
			}
			} catch (CommandInvalidException e){
				log.info("moveAsync  Exception ======" +e.toString());
			}
			
			Frame frameCube = tool.getDefaultMotionFrame().copy();						
			log.info("frame cube InWorld" + frameCube.toStringInWorld());
			
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType("Cube");		
			ret.setResultMsg("Cube ok");
		///////////////////////
			JointPosition jtPos = m_robot.getCurrentJointPosition();
			Frame frm = m_robot.getCurrentCartesianPosition(m_robot.getFlange());	
			Param  prm = new Param();
			prm.x = frm.getX();
			prm.y = frm.getY();
			prm.z = frm.getZ();
			prm.a = Math.toDegrees(frm.getAlphaRad());
			prm.b = Math.toDegrees(frm.getBetaRad());
			prm.c = Math.toDegrees(frm.getGammaRad());
		    ret.setParam(prm);
			ret.setResultMsg(jtPos.toString());
			return ret;
		}
		else if (opType.equals("Cube_origin"))
		{		
			double[] joint = bean.getJointPos();
			JointPosition jp = new JointPosition(
					Math.toRadians(joint[0]),
					Math.toRadians(joint[1]),
					Math.toRadians(joint[2]),
					Math.toRadians(joint[3]),
					Math.toRadians(joint[4]),
					Math.toRadians(joint[5]),
					Math.toRadians(joint[6]));
		    mc = m_robot.moveAsync(ptp(jp).setJointVelocityRel(0.3* speedLevel).setJointAccelerationRel(0.02));
			while (!mc.isFinished()) {
				ProtocolResult ret = checkStop();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}				
				ret = checkTorque();				
				if (ret != null) {
					log.info("ret != null  " );		
					return ret;
				}		
			}
			log.info("Cube_origin " );
			//////////////////		
			Frame frameCube = tool.getDefaultMotionFrame().copy();		
			log.info("frameCube tool " + frameCube.toString());	
			
			frameOrigin = tool.getDefaultMotionFrame().copy(World.Current.getRootFrame());		
			log.info("frame cube Origin" + frameOrigin.toString());
			log.info("frame cube Origin InWorld" + frameOrigin.toStringInWorld());
				
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);		
			ret.setResultMsg("Cube_origin ok");
			return ret;
		}
		else if (opType.equals("Master"))
		{
			if (isSoftMode) {
			SoftModeOff();
			log.info("Master SoftModeOff");
			}
			return funcMaster();
		}
		//add 2021-08-18 jinwang ,dont't delete;
		else if (opType.equals("GetUserToolPos"))
		{
			//RobotUserTool;
			ObjectFrame rut = tool.getFrame("RobotUserTool");
			JointPosition jtPos = m_robot.getCurrentJointPosition();
			//Frame frm = m_robot.getCurrentCartesianPosition(rut);
			Frame frm = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);	
			Param  prm = new Param();
			prm.x = frm.getX();
			prm.y = frm.getY();
			prm.z = frm.getZ();
			prm.a = Math.toDegrees(frm.getAlphaRad());
			prm.b = Math.toDegrees(frm.getBetaRad());
			prm.c = Math.toDegrees(frm.getGammaRad());
		    ret.setParam(prm);
			ret.setResultMsg(jtPos.toString());
			return ret;
		}
		else if (opType.equals("GetJointPos"))
		{
			JointPosition jtPos = m_robot.getCurrentJointPosition();
			Frame frm = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);	
			Param  prm = new Param();
			prm.x = frm.getX();
			prm.y = frm.getY();
			prm.z = frm.getZ();
			prm.a = Math.toDegrees(frm.getAlphaRad());
			prm.b = Math.toDegrees(frm.getBetaRad());
			prm.c = Math.toDegrees(frm.getGammaRad());
		    ret.setParam(prm);
			ret.setResultMsg(jtPos.toString());
			return ret;
		}
		else if (opType.equals("Speed"))
		{
			String str = bean.getParam().getTarget();
			speedLevel = Double.valueOf(str) / 100;
			log.info("speedLevel " + speedLevel);
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);		
			ret.setResultMsg("Speed ok");
			return ret;
		}
		else if (opType.equals("Load"))
		{
			log.info("**** Mass " + tool.getLoadData().toString());
			
			String[] strArray =  bean.getParam().getTarget().split(";");
		 	double MASS = new Double(strArray[0]);
		 	String[] strMass = strArray[1].split(",");
		 	double[] CENTER_MASS = {0, 0, 0};
		 	CENTER_MASS[0] = new Double(strMass[0]);
		 	CENTER_MASS[1] = new Double(strMass[1]);
		 	CENTER_MASS[2] = new Double(strMass[2]);

			log.info("CENTER_MASS " + CENTER_MASS[0]);
			log.info("CENTER_MASS " + CENTER_MASS[1]);
			log.info("CENTER_MASS " + CENTER_MASS[2]);

			LoadData load = tool.getLoadData();
			
			load.setMass(MASS);		
			load.setCenterOfMass(
					CENTER_MASS[0],
					CENTER_MASS[1],
					CENTER_MASS[2]);

			log.info("**** Mass " + tool.getLoadData().toString());
			mc = m_robot.move(
					positionHold(posMode, 1, TimeUnit.SECONDS));
			
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);		
			ret.setResultMsg("Load ok");
			return ret;
		}
		else if (opType.equals("CheckBrakeOK"))
		{
			boolean ok = m_brakeTestExecutor.isBrakeTestOK();

			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);
			ret.setResultMsg("CheckBrakeOK");
			int code = ok ? 0 : -1;
			log.info("CheckBrakeOK " + ok);
			log.info("CheckBrakeOK code " + code);
			ret.setResultCode(code);
			
			return ret;
		
		}
		else if (opType.equals("CheckNearPos"))
		{
			double[] joint = bean.getJointPos();
			JointPosition jp = new JointPosition(
					Math.toRadians(joint[0]),
					Math.toRadians(joint[1]),
					Math.toRadians(joint[2]),
					Math.toRadians(joint[3]),
					Math.toRadians(joint[4]),
					Math.toRadians(joint[5]),
					Math.toRadians(joint[6]));
			
			boolean ok = m_robot.getCurrentJointPosition().isNearlyEqual(jp, Math.toRadians(2));
		
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);
			ret.setResultMsg("CheckNearPos");
			int code = ok ? 0 : -1;
			ret.setResultCode(code);
			
			return ret;
		}
		else if (opType.equals("PGMS"))
		{
			log.info("PGMS ");
			pgms.doReferencing();
			ProtocolResult ret = new ProtocolResult();
			ret.setOperateType(opType);		
			ret.setResultMsg("PGMS ok");
			return ret;
		}
		else {
			log.info(" cmd not found !" + opType);
			return null;	
		}
		return null;	
	}
	
	public ProtocolResult SoftModeOff()
	{
		mc.cancel();	
		isSoftMode = false;
		m_robot.moveAsync(positionHold(posMode, 1, TimeUnit.SECONDS));
		
		ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("SoftMode_Off");		
		ret.setResultMsg("SoftMode_Off ok");
		return ret;
	}
	
	//Ã–Ã˜Ã–ÃƒÂ¡Â¢Â¹Ã©ÃŽÂ»ÂµÂ½Â¾Â­ÂµÃ¤ÃŽÂ»Ã–Ãƒ
	private ProtocolResult Reset(ProtocolBean bean)
	{
		JointPosition jp_02 = new JointPosition(0, 0, 0, Math.toRadians(90), 0, Math.toRadians(-90), 0);
		
		m_robot.setHomePosition(jp_02);
		m_robot.move(ptpHome().setJointVelocityRel(0.6));
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(0);
		
		return result;
	}
		
	private ProtocolResult StartDrawCircle(ProtocolBean bean)
	{	
		if (m_drawCircle == null)
		{
			m_drawCircle = new DrawCircleEx(m_robot);
			m_drawCircle.initialize();
			m_drawCircle.setProcess(this);
		}
		else
		{
			m_drawCircle.initialize();
		}
				
		ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("StartDrawCircle");	
		ret.setResultCode(0);
		
		return ret;
	}
	
	private ProtocolResult DrawCircle(ProtocolBean bean)
	{	
		if (m_drawCircle == null)
		{
			ProtocolResult ret = new ProtocolResult();
			ret.setResultCode(1);
			ret.setResultMsg("not start");
			return ret;
		}
				
		ProtocolResult ret = m_drawCircle.run();	
		ret.setOperateType("DrawCircle");		
		ret.setResultMsg(ret.getResultCode() == 1 ? "ç”»åœ†ä»»åŠ¡å·²å®Œæˆ�" : "");
		
		return ret;
	}
	
	private ProtocolResult freeHand()
	{
	
		if( m_freehandMotion == null)
		{
			m_freehandMotion = new FreeHandMotion(m_robot);
		}
		
		return m_freehandMotion.run();
	}
	
	public ProtocolResult StartDragLine(ProtocolBean bean)
	{
		/*if (m_drawLine == null) 
	    {
			m_drawLine = new DrawLine(m_robot, m_observerMgr, bean.getParam(), LINE_MODEL.FORCE_LINE_EX);
			m_drawLine.setStepping(0.2);
			m_drawLine.StartLisener();
	    }*/	
		
		if (bean == null || bean.getParam() == null) return null;
		
		if (m_dragLineOnY == null) 
	    {
			m_dragLineOnY = new DragLineOnY(m_robot, m_observerMgr, bean.getParam());
			m_dragLineOnY.setStepping(2);
			//m_dragLineOnY.StartLisener();
	    }		
		
		if (null == m_servoLineMotion)
		{
			m_servoLineMotion = new SmartServoLINSimpleMotion(m_robot);
		}		
		
		m_servoLineMotion.setDrawLine(m_dragLineOnY);
		m_servoLineMotion.setDistanceAndDstPosition(bean.getParam().getY(), bean.getParam2().getX(), bean.getParam2().getY(), bean.getParam2().getZ());
		boolean ok = m_servoLineMotion.initialize();
		//m_drawLine.BindServoLineMotion(m_servoLineMotion);
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(ok ? 0 : 1);
		result.setResultMsg("ç›®æ ‡ä¸�å�¯è¾¾");
		
		return result;
	}
	
	public ProtocolResult StartDragLineManual(ProtocolBean bean)
	{				
		if (bean == null || bean.getParam() == null || bean.getParam2() == null) return null;
						
		boolean ok = false;
		String msg = "";
		if (null == m_servoLineMotion)
		{
			m_servoLineMotion = new SmartServoLINSimpleMotion(m_robot);
			m_servoLineMotion.setDistanceAndDstPosition(bean.getParam().getY(), bean.getParam2().getX(), bean.getParam2().getY(), bean.getParam2().getZ());
			ok = m_servoLineMotion.initialize();
			if (!ok)
			{
				msg = "ç›®æ ‡ä¸�å�¯è¾¾";
			}			
		}		
		else
		{
			// å¤–éƒ¨1ç§’è°ƒç”¨ä¸€æ¬¡
			//msg = m_servoLineMotion.startSineMovement_distance();
		}		
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(ok ? 0 : 1);
		result.setResultMsg(msg);
		
		return result;
	}
	
	public ProtocolResult ServoMove(ProtocolBean bean)
	{
		if (bean == null || bean.getParam() == null) return null;
		
		boolean ok = false;
		String msg = "";
		if (null == m_servoLineMotion)
		{
			m_servoLineMotion = new SmartServoLINSimpleMotion(m_robot);
			ok = m_servoLineMotion.initialize();					
		}		
		else
		{
			// å¤–éƒ¨å�‘ç”Ÿå�˜åŒ–æ—¶è°ƒç”¨ä¸€æ¬¡
			//msg = m_servoLineMotion.startSineMovement_move(bean.getParam().getX(), bean.getParam().getY(), bean.getParam().getZ());
		}		
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(ok ? 0 : 1);
		result.setResultMsg(msg);
		
		return result;
	}
	
	public ProtocolResult EndDragLine(ProtocolBean bean)
	{
		if (m_drawLine != null)
		{
			m_drawLine.StopListener();
		}
		
		ProtocolResult result = null;
		return result;
	}
	
	public ProtocolResult StartDrawLine(ProtocolBean bean)
	{
		DrawLine drawLine = new DrawLine(m_robot, m_observerMgr, bean.getParam(), LINE_MODEL.NONE);
		
		drawLine.setStepping(5);
		drawLine.StartLisener();
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(0);
		
		return result;
	}
	
	public ProtocolResult StartDrawLineNormal(ProtocolBean bean)
	{
		DrawLine drawLine = new DrawLine(m_robot, m_observerMgr, bean.getParam(), LINE_MODEL.NONE);
		
		drawLine.drawLineNormal();
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(0);
		
		return result;
	}	
	
	public ProtocolResult StartLisener()
	{
		if (m_testDrag == null) m_testDrag = new TestDrag(m_robot, m_observerMgr); 
		
		m_testDrag.StartLisener();
		
		ProtocolResult result = null;
		return result;
	}
	
	// å®žæ—¶è·Ÿè¸ª
	public ProtocolResult StartPositionLisener(ProtocolBean bean)
	{
		log.info("StartPositionLisener");
		if (m_sslNormal == null) 
		{
			m_sslNormal = new SmartServoLINNormal(m_robot); 
		}
		else
		{
			if (bean.getParam() != null)
			{
				m_sslNormal.setNewPoint(
						bean.getParam().getX(), 
						bean.getParam().getY(), 
						bean.getParam().getZ());
			}			
		}
		
		ProtocolResult result = new ProtocolResult();
		result.setResultCode(0);
		
		return result;
	}
	
	public ProtocolResult StopPositionLisener()
	{
		if (m_sslNormal != null) 
		{
			m_sslNormal.StopMotion(); 
		}		
		
		ProtocolResult result = null;
		return result;
	}
	
	public ProtocolResult StopListener()
	{
		if (m_testDrag != null)
		{
			m_testDrag.StopListener();
		}
		
		ProtocolResult result = null;
		return result;
	}
	
	public void Test(ProtocolBean bean)
	{
		if (m_drawLine == null) m_drawLine = new DrawLine(m_robot, m_observerMgr, bean.getParam(), LINE_MODEL.NORMAL_LINE);
		
		m_drawLine.test();
	}
	
	public void IsPathViaSingularPoint()
	{	
		//return;
		
		com.kuka.roboticsAPI.geometricModel.Frame currentFrame = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
		if(Functions.isPathViaSingularPoint(m_robot, 0, 0, 200)){
			log.info("goal can not be reached!");
		}else{
			//m_robot.move(linRel(100, 200, 300, 0, 0, 0).setJointVelocityRel(0.2));
			currentFrame = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
			currentFrame.setX(currentFrame.getX() + 0);
			currentFrame.setY(currentFrame.getY() + 0);
			currentFrame.setZ(currentFrame.getZ() + 200);
			m_robot.move(lin(currentFrame).setJointVelocityRel(0.2));
		}
		
	}
	
	public void IsPathViaSingularPoint2()
	{		
		com.kuka.roboticsAPI.geometricModel.Frame currentFrame = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
		
		if(Functions.isPathViaSingularPoint(m_robot, 0, 0, -100)){
			log.info("goal can not be reached!");
		}else{
			currentFrame = m_robot.getCurrentCartesianPosition(m_robot.getFlange());
			currentFrame.setZ(currentFrame.getZ() - 100);
			m_robot.move(lin(currentFrame).setJointVelocityRel(0.2));
		}
	}
	
	public ProtocolResult TestBrake(ProtocolBean bean)
	{
		if (m_brakeTestExecutor == null) return null;
		
		return m_brakeTestExecutor.run();
	}
	
	public void TestBatch(ProtocolBean bean)
	{
		if (m_testBatch == null)
		{
			m_testBatch = new TestBatch(m_robot);
			m_testBatch.init();
		}
		else
		{
			m_testBatch.run();
		}
		
	}
	
	public void exit()
	{
		if (m_servoLineMotion != null)
		{
			//m_servoLineMotion.StopMotion();
		}
	}
	
	
	public void jogAxis(int idx, double jogPos)
	{		
		// ä½�ç½®å�‡å¼§åº¦
		JointPosition jtPos = m_robot.getCurrentJointPosition();
		double[] axises = jtPos.get();
		double jogVec = 1;
		jogPos = Math.toRadians(jogPos);

		if (axises[idx] < jogPos) {
			jogVec = 1;
			// isJog = true;
		} else if (axises[idx] > jogPos) {
			jogVec = -1;
			// isJog = true;
		}
		log.info("Jog axis " + " " + idx);
		JointEnum jtNum = JointEnum.values()[idx];
		try {
			jogging.startJointJogging(EnumSet.of(jtNum), 0, -1);
			log.info("start Joint Jogging...");
			while (true) {
				jtPos = m_robot.getCurrentJointPosition();
				axises = jtPos.get();
				if (Math.abs(axises[idx] - jogPos) < 0.02) {
					jogging.stopJointJogging(EnumSet.of(jtNum));
					break;
				}
				jogging.updateJointJogging(EnumSet.of(jtNum), jogVec);

				ThreadUtil.milliSleep(100);
			}
		} catch (Exception e) {
			log.error("Jogging failed... " + e.toString());
		}
	}
	
	
public ProtocolResult funcMaster() {
		ThreadUtil.milliSleep(5000); 
		ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("Master");
		
		double jogPosArray[] = {0, -60, 0, 0, 0, 0, 0}; // è§’åº¦
		for (int idx = 0; idx < 7; ++idx) {
			if (!mastering.isAxisMastered(idx)) {
				log.info("Mastering Axis " + idx + "...");
				jogAxis(idx, 0);
				if (mastering.masterAxis(idx)) {
					log.info("Mastering Axis " + idx + " finished");
				} else {
					log.error("Mastering Axis " + idx + " failed");
					ret.setResultCode(-1);		
					ret.setResultMsg("Master bad");
					return ret;
				}
			}
			jogAxis(idx, jogPosArray[idx]);
		}

		ret.setResultCode(0);	
		ret.setResultMsg("Master ok");
		return ret;
	}
	
public void jog2AxisPosition(double[] axisesDest /*è§’åº¦ */) {
	JointPosition jtPos = m_robot.getCurrentJointPosition();
	double[] axises = jtPos.get();

	int cnt = 0;
	while (++cnt <= 60) {
		JointPosition jtPosAA = m_robot.getCurrentJointPosition();
		double[] axisesTmp = jtPosAA.get();
		for (int idx = 0; idx < 7; ++idx) {
			double delta = (axises[idx] - Math.toRadians(axisesDest[idx]))
					* cnt / 10;
			if (Math.abs(axisesTmp[idx] - Math.toRadians(axisesDest[idx])) > 0.1) {
				jogAxis(idx, Math.toDegrees(axises[idx] - delta));
			}
		}
	}
}
	
	public ProtocolResult funcMasterNew() {
		ThreadUtil.milliSleep(10000); // ç­‰å¾…10s ä½¿ç¨‹åº�æ�¢å¤�è¿�è¡ŒçŠ¶æ€�
	
		ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("Master");

		JointPosition jtPos = m_robot.getCurrentJointPosition();
		double[] axises = jtPos.get();
		log.info(" ========== " + axises);

		int cnt = 0;
		if (axises[1] > 0) {
			while (++cnt <= 20) {
				JointPosition jtPosAA = m_robot.getCurrentJointPosition();
				double[] axisesTmp = jtPosAA.get();
				if (axises[1] > Math.toRadians(60))
					break;
				double delta = (axises[1] - (axises[1] - Math.toRadians(20)))
						* cnt / 20;
				if (Math.abs(axisesTmp[1] - (axises[1] - Math.toRadians(20))) > 0.1) {
					jogAxis(1, Math.toDegrees(axises[1] - delta));
				}
			}
			log.info("Move jog Axis 2 ===============...");
			double[] axisesDest = { 0, 50, 0, -70, 0, -30, 0 };
			jog2AxisPosition(axisesDest);
			
			// è°ƒæ•´ 2ï¼Œ4ï¼Œ6è½´ä½�ç½®
			double[] axisesDest246 = { 0, 85, 0, 0, 0, 0, 0 };
			jog2AxisPosition(axisesDest246);

			for (int idx = 2; idx < 7; ++idx) {
				if (!mastering.isAxisMastered(idx)) {
					log.info("Mastering Axis " + idx + "...");
					jogAxis(idx, 0);
					if (mastering.masterAxis(idx)) {
						log.info("Mastering Axis " + idx + " finished");
					} else {
						log.error("Mastering Axis " + idx + " failed");
						ret.setResultCode(-1);		
						ret.setResultMsg("Master bad");
						return ret;
					}
				}
			}
			
		}
		// ================================
	
		jtPos = m_robot.getCurrentJointPosition();
		axises = jtPos.get();
		log.info("aaaaaaaaaa  ==********************======== " + axises);
		cnt = 0;
		// ä¼¸ç›´2è½´ï¼Œ å¼¯æ›²4ï¼Œ6è½´
		double[] axisesDestWind = { 0, 0, 0, -90, 0, 10, 0 };
		jog2AxisPosition(axisesDestWind);
		
		for (int idx = 0; idx < 2; ++idx) {
			if (!mastering.isAxisMastered(idx)) {
				log.info("Mastering Axis " + idx + "...");
				jogAxis(idx, 0);
				if (mastering.masterAxis(idx)) {
					log.info("Mastering Axis " + idx + " finished");
				} else {
					log.error("Mastering Axis " + idx + " failed");
					ret.setResultCode(-1);		
					ret.setResultMsg("Master bad");
					return ret;
				}
			}
		}

		ret.setResultCode(0);		
		ret.setResultMsg("Master ok");
		return ret;
	}
	

}
