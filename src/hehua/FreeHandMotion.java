package hehua;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptpHome;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.handGuiding;

import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class FreeHandMotion {
	private LBRMed robot_ = null;
	
	public FreeHandMotion(LBRMed rb)
	{
		robot_ = rb;
	}
	
	public void initialize() {
		
	}
	
	public ProtocolResult run() {
		
		robot_.move(handGuiding().setJointVelocityLimit(1.0)
				.setCartVelocityLimit(2500)
				.setJointLimitViolationFreezesAll(false));
		
		ProtocolResult ret = new ProtocolResult();
		ret.setOperateType("freeHand");		
		ret.setResultMsg("freehand motion succeed");
		return ret;
		
	}
}
