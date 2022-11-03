package functions;

import com.kuka.med.deviceModel.LBRMed;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.geometricModel.Frame;

public class Functions{
	private static final double ADMIT_TOLERANCE = 2;
	private static boolean isInAxisLimits(JointPosition pos){
		return (Math.toDegrees(pos.get(JointEnum.J1)) > -170 && Math.toDegrees(pos.get(JointEnum.J1)) < 170) && 
				(Math.toDegrees(pos.get(JointEnum.J2)) > -120 && Math.toDegrees(pos.get(JointEnum.J2)) < 120) &&
				(Math.toDegrees(pos.get(JointEnum.J3)) > -170 && Math.toDegrees(pos.get(JointEnum.J3)) < 170) &&
				(Math.toDegrees(pos.get(JointEnum.J4)) > -120 && Math.toDegrees(pos.get(JointEnum.J4)) < 120) &&
				(Math.toDegrees(pos.get(JointEnum.J5)) > -170 && Math.toDegrees(pos.get(JointEnum.J5)) < 170) &&
				(Math.toDegrees(pos.get(JointEnum.J6)) > -120 && Math.toDegrees(pos.get(JointEnum.J6)) < 120) &&
				(Math.toDegrees(pos.get(JointEnum.J7)) > -175 && Math.toDegrees(pos.get(JointEnum.J7)) < 175);
	}
	
	public static boolean isPathViaSingularPoint(LBRMed robot, double x, double y, double z, double stride){
		double scaleX;
		double scaleY;
		double scaleZ;
		Frame currentFrame = robot.getCurrentCartesianPosition(robot.getFlange());
		if(x != 0){
			scaleX = 1;
			scaleY = y / x;
			scaleZ = z / x;
		}else if(y != 0){
			scaleX = 0;
			scaleY = 1;
			scaleZ = z / x;
		}else{
			scaleX = 0;
			scaleY = 0;
			scaleZ = 1;
		}
		
		double index = 0;
		int errorTimes = 0;
		while(index < (1 / stride)){
			currentFrame.setX(currentFrame.getX() + stride * x * scaleX);
			currentFrame.setY(currentFrame.getY() + stride * y * scaleY);
			currentFrame.setZ(currentFrame.getZ() + stride * z * scaleZ);
			
			try
			{
			JointPosition currentPos = robot.getInverseKinematicFromFrameAndRedundancy(currentFrame);
			System.out.println(Math.toDegrees(currentPos.get(JointEnum.J4)));
			if(!isInAxisLimits(currentPos)){
				System.out.println("over limits!");
				return true;
			}
			if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J4))) < ADMIT_TOLERANCE){
//				ç¬¬ä¸€ç±»å¥‡å¼‚ç‚¹: A4 = 0
				System.out.println("condition 1");
				return true;
			}
			if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J4) - 90)) < ADMIT_TOLERANCE &&
					Math.abs(Math.toDegrees(currentPos.get(JointEnum.J6))) < ADMIT_TOLERANCE){
//				ç¬¬äºŒç±»å¥‡å¼‚ç‚¹ï¼šA4 = 90 && A6 = 0ï¼›
				System.out.println("condition 2");
				return true;
			}
			if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J2))) < ADMIT_TOLERANCE &&
					(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J3)) - 90) < ADMIT_TOLERANCE || 
					Math.abs(Math.toDegrees(currentPos.get(JointEnum.J3)) + 90) < ADMIT_TOLERANCE)){
				System.out.println("condition 3");
				return true;
			}
			if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J6))) < ADMIT_TOLERANCE &&
					(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J5)) - 90) < ADMIT_TOLERANCE || 
							Math.abs(Math.toDegrees(currentPos.get(JointEnum.J5)) + 90) < ADMIT_TOLERANCE)){
				System.out.println("condition 4");
				return true;
			}
			}
			catch (Exception e)
			{
				errorTimes++;
			}
			
			index++;
		}
		System.out.println("condition 5 times " + errorTimes);
//		index += stride;
		return (errorTimes >= 1);
	}	
	
	public static boolean isPathViaSingularPoint(LBRMed robot, double x, double y, double z){
//		å¦‚æžœæ²¡æœ‰è¾“å…¥æ­¥å¹…å�‚æ•°ï¼Œé‚£ä¹ˆé»˜è®¤æ­¥é•¿ä¸º0.1mm
		return isPathViaSingularPoint(robot, x, y, z, 0.1);
	}
}