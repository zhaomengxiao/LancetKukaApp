package application;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;

import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.task.ITaskLogger;

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
public class WallTest extends RoboticsAPIApplication {
	@Inject
	private LBR lbr;

	@Inject
	private ITaskLogger log;

	@Named("Tool_2")
	@Inject
	private Tool tool;
	
	private double fx;
	private double px;
	private double dfx;
	private double fy;
	private double py;
	private double dfy;
	private double fz;
	private double pz;
	private double dfz;

	private double cx;
	private double cy;
	private double cz;

	private double ccx;
	private double ccy;
	private double ccz;

    private Tool _toolAttachedToLBR;
    private LoadData _loadData;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 0 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 0 };

	
	private CartesianImpedanceControlMode cic;
	boolean bstop = false;

	@Override
	public void initialize() {
		// initialize your application here

		_loadData = new LoadData();
        _loadData.setMass(1.93);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(lbr.getFlange());
		
        cic = new CartesianImpedanceControlMode();
        cic.parametrize(CartDOF.TRANSL).setStiffness(2000);
	}

	@Override
	public void run() {
		// your application execution starts here
		// lbr.move(ptpHome());
		JointPosition actPos = lbr.getCurrentJointPosition();
		try{
			
			Frame Object5 = lbr.getCurrentCartesianPosition(lbr.getFlange());
			Object5.setX(0);
			Object5.setY(0);
			Object5.setZ(0);
			Object5.setAlphaRad(0);
			Object5.setBetaRad(0);
			Object5.setGammaRad(0);
//			lbr.getInverseKinematicFromFrameAndRedundancy(Object4);
			System.out.println("ss2");
			try{
			actPos=lbr.getInverseKinematicFromFrameAndRedundancy(Object5);
			}
			catch (IllegalArgumentException e)
			{
				System.out.println("ss3");
			} 
			System.out.println("ss1");
//			System.out.println("J1："+Math.toDegrees(test.get(JointEnum.J1))+"   J2:"+Math.toDegrees(test.get(JointEnum.J2))+"   J3:"+Math.toDegrees(test.get(JointEnum.J3))+"   J4:"+Math.toDegrees(test.get(JointEnum.J4))+"   J5:"+Math.toDegrees(test.get(JointEnum.J5))+"   J6:"+Math.toDegrees(test.get(JointEnum.J6))+"   J7:"+Math.toDegrees(test.get(JointEnum.J7)) );
			System.out.println("ss");
		}
		catch (IllegalArgumentException e)
		{
			System.out.println("ss3");
		} 
//		   if (!ServoMotion.validateForImpedanceMode(lbr))
//	        {
//	            getLogger()
//	                    .info("Validation of torque model failed - correct your mass property settings");
//	            getLogger()
//	                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
//	        }
//
//		lbr.move(ptp(getFrame("/P2")).setJointVelocityRel(0.5));
//
//		Frame current = lbr.getCurrentCartesianPosition(lbr.getFlange());
//		SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(current);
//		aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
//
//		log.info("Start Smart Servo Lin motion");
//
//		 _toolAttachedToLBR.moveAsync(aSmartServoLINMotion.setMode(cic));
//
//		getLogger().info("Get the runtime of the SmartServoLIN motion");
//		ISmartServoLINRuntime smartServoLINRuntime = aSmartServoLINMotion
//				.getRuntime();
//
//		px = py = pz = 0.2;
//
//		Frame destFrame = current.copyWithRedundancy();
//
//		double zFx = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//				.getX();
//		double zFy = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//				.getY();
//		double zFz = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//				.getZ();
//
//		ccx = lbr.getCurrentCartesianPosition(lbr.getFlange()).getX();
//		ccy = lbr.getCurrentCartesianPosition(lbr.getFlange()).getY();
//		ccz = lbr.getCurrentCartesianPosition(lbr.getFlange()).getZ();
//
//		log.info("get Force in x " + zFx);
//		log.info("get Force in y " + zFy);
//		log.info("get Force in z " + zFz);
//
//		try {
//			while (!bstop) {
//
//				fx = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//						.getX();
//				fy = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//						.getY();
//				fz = lbr.getExternalForceTorque(lbr.getFlange()).getForce()
//						.getZ();
//				
//				dfx = -1 * px * (fx - zFx);
//				dfy = +1 * py * (fy - zFy);
//				dfz = -1 * pz * (fz - zFz);
//
//				cx = lbr.getCurrentCartesianPosition(lbr.getFlange()).getX();
//				cy = lbr.getCurrentCartesianPosition(lbr.getFlange()).getY();
//				cz = lbr.getCurrentCartesianPosition(lbr.getFlange()).getZ();
//
//				if(Math.abs(ccx-cx)>50 && (ccx-cx)*fx>0 || Math.abs(dfx) < 0.5){
//					dfx = 0;
//				}
//				if(Math.abs(ccy-cy)>50 && (ccy-cy)*fy<0 || Math.abs(dfy) < 0.5){
//					dfy = 0;
//				}
//				if(Math.abs(ccz-cz)>50 && (ccz-cz)*fz>0 || Math.abs(dfz) < 0.5){
//					dfz = 0;
//				}
//				
////				if (Math.abs(dfx) < 0.5) {
////					dfx = 0;
////				}
////				if (ccx - cx > 50) {
////					if (fx > 0) {
////						dfx = 0;
////
////					}
////				}
////
////				if (ccx - cx < -50) {
////					if (fx < 0) {
////						dfx = 0;
////
////					}
////				}
//
////				if (Math.abs(dfy) < 0.5) {
////					dfy = 0;
////				}
////				if (ccy - cy >= 50) {
////					if (fy < 0) {
////						dfy = 0;
////
////					}
////				}
////				if (ccy - cy <= -50) {
////					if (fy > 0) {
////						dfy = 0;
////
////					}
////				}
//
//				if (Math.abs(dfz) < 0.5) {
//					dfz = 0;
//				}
//				if (ccz - cz > 50) {
//					if (fz > 0) {
//						dfz = 0;
//
//					}
//				}
//
//				if (ccz - cz < -50) {
//					if (fz < 0) {
//						dfz = 0;
//
//					}
//
//				}
//
//				// log.info("get Force in delta x" + dfx + ",y " + dfy + ",z " +
//				// dfz);
//				destFrame.setX(destFrame.getX() + dfx);
//				destFrame.setY(destFrame.getY() + dfy);
//				destFrame.setZ(destFrame.getZ() + dfz);
//
//				smartServoLINRuntime.updateWithRealtimeSystem();
//				
//				smartServoLINRuntime.setDestination(destFrame);
//
//				smartServoLINRuntime.changeControlModeSettings(cic);
//				if (lbr.getCurrentCartesianPosition(lbr.getFlange()).getX() > 700) {
////					bstop = true;
//				}
//
//				ThreadUtil.milliSleep(5);
//			}
//
//		} catch (Exception e) {
//			// TODO: handle exception
//			log.info(e.toString());
//		}
//
//		smartServoLINRuntime.stopMotion();

	}

	@Override
	public void dispose() {
		// TODO Auto-generated method stub
		bstop = true;
		super.dispose();
	}
}