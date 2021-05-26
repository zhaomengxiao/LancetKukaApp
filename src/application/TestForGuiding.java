package application;


import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

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
public class TestForGuiding extends RoboticsAPIApplication {
	@Inject
	private LBRMed lbr;
	@Named("Tool_2")
	
	@Inject
	private Tool needle;
	
	
	@Override
	public void initialize() {
		// initialize your application here
		needle.attachTo(lbr.getFlange());
	}

	@Override
	public void run() {
		// your application execution starts here
		// = lbr.getCurrentCartesianPosition(lbr.getFlange());
		Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
		Frame Object=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
		int num=0;
		double nMinA=0,nMinB=0,nMinC=0;
		double nMinSun=10000000;
		double nObjectA=10000,nObjectB=100000,nObjectC=100000;

		for (num = 0; num < 360; num = num + 1){
			//当前点位
			Frame destObject = current.setX(-501.36);
			destObject = Object.setY(4.7);
			destObject = Object.setZ(386.16);
			destObject = Object.setGammaRad(Math.toRadians(138.05));		
			destObject = Object.setBetaRad(Math.toRadians(-16.99));
			destObject = Object.setAlphaRad(Math.toRadians(154.78));
			
			//System.out.println(Math.toDegrees(destObject.getBetaRad()));
			
			
			//目标点位
			Frame destFrame = current.setX(-714.741);
			destFrame = current.setY(144.732);
			destFrame = current.setZ(315.285);
			destFrame = current.setGammaRad(Math.toRadians(154.836));		
			destFrame = current.setBetaRad(Math.toRadians(39.7936));
			destFrame = current.setAlphaRad(Math.toRadians(-139.612));
			
			
//			System.out.println("11"+destFrame);
//			
//			destFrame = current.setX(-714.741);
//			destFrame = current.setY(144.732);
//			destFrame = current.setZ(315.285);
//			destFrame = current.setAlphaRad(Math.toRadians(-139.612));
//			destFrame = current.setBetaRad(Math.toRadians(39.7936));
//			destFrame = current.setGammaRad(Math.toRadians(154.836));	
//			System.out.println("22"+destFrame);
			
			
			Frame Ptest1 = destFrame.transform((Transformation.ofDeg(0, 0, 0, num, 0, 0)));
			
			nMinA=Math.abs(Math.abs(Math.toDegrees(destObject.getAlphaRad()))-Math.abs(Math.toDegrees(Ptest1.getAlphaRad())));
			nMinB=Math.abs(Math.abs(Math.toDegrees(destObject.getBetaRad()))-Math.abs(Math.toDegrees(Ptest1.getBetaRad())));
			nMinC=Math.abs(Math.abs(Math.toDegrees(destObject.getGammaRad()))-Math.abs(Math.toDegrees(Ptest1.getGammaRad())));
			
//			nMinA=Math.abs(Math.toDegrees(destObject.getAlphaRad())-Math.toDegrees(Ptest1.getAlphaRad()));
//			nMinB=Math.abs(Math.toDegrees(destObject.getBetaRad())-Math.toDegrees(destObject.getBetaRad()));
//			nMinC=Math.abs(Math.toDegrees(destObject.getGammaRad())-Math.toDegrees(Ptest1.getGammaRad()));
			
			//System.out.println(Math.toDegrees(destObject.getAlphaRad()));
			//System.out.println(Math.toDegrees(Ptest1.getAlphaRad()));
			
			//System.out.println("a:"+nMinA+" b:"+nMinB+" c:"+nMinC);
			if((nMinA+nMinB+nMinC)<nMinSun)
			{
				
				nMinSun=(nMinA+nMinB+nMinC);
				nObjectA=Math.toDegrees(Ptest1.getAlphaRad());
				nObjectB=Math.toDegrees(Ptest1.getBetaRad());
				nObjectC=Math.toDegrees(Ptest1.getGammaRad());
			}
		
			//ThreadUtil.milliSleep(10);
		}
		System.out.println((nMinSun));
		//System.out.println("a:"+Math.toDegrees(Ptest1.getAlphaRad())+" b:"+Math.toDegrees(Ptest1.getBetaRad())+" c:"+Math.toDegrees(Ptest1.getGammaRad()));
		System.out.println("a:"+nObjectA+" b:"+nObjectB+" c:"+nObjectC);
		
	}
}