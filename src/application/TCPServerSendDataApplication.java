package application;



import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.positionHold;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.math.BigDecimal;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.StatisticTimer;
import com.kuka.common.ThreadUtil;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.connectivity.motionModel.directServo.DirectServo;
import com.kuka.connectivity.motionModel.directServo.IDirectServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.generated.ioAccess.MytestIOIOGroup;
import com.kuka.generated.ioAccess.SafeDataIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPITask;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.conditionModel.BooleanIOCondition;
import com.kuka.roboticsAPI.controllerModel.Controller;
import com.kuka.roboticsAPI.deviceModel.CartesianVelocityLimitInfo;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.executionModel.ExecutionState;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.ITransformation;
import com.kuka.roboticsAPI.geometricModel.math.Matrix;
import com.kuka.roboticsAPI.geometricModel.math.MatrixTransformation;
import com.kuka.roboticsAPI.geometricModel.math.Rotation;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;
import com.kuka.roboticsAPI.geometricModel.math.Vector;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.HandGuidingMotion;
import com.kuka.roboticsAPI.motionModel.IMotion;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

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
public class TCPServerSendDataApplication extends RoboticsAPIApplication {
//    int i=0;
	private HandGuidingMotion motion;

	@Inject
	private BrakeTestMonitorSampleApp BreakTest;
	
	@Inject
	private motion2 OnlyPlane;
	
  
	@Inject
	private  LBR lbr;
	private Tool _toolAttachedToLBR;
	private Controller kuka_Sunrise_Cabinet_1;
    @Inject
    private MytestIOIOGroup io;
    private static final int AIM = 10;
	private static int i = 0;
	private CartesianImpedanceControlMode _nullspacemode ;
	IMotion bHandGuidingStart;
	ServerSocket serverSocket=null;
	ServerSocket serverSocketSend=null;
	DataOutputStream outputStream=null;
	OutputStreamWriter writer=null;
	OutputStreamWriter writer_recive=null;
	Socket socket=null;
	Socket socket_recive=null;
	DataOutputStream outputStream_recive=null;
	
	 private static final double FREQENCY = 0.1;
	 private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
	 private LoadData _loadData;
	 private static final double[] TRANSLATION_OF_TOOL = { -150.7, 0, 227.9 };
	 private static final double MASS = 0;
	 private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 35.3, 0, 101.3 };
	 private static final String TOOL_FRAME = "toolFrame";
     private static final double[] MAX_TRANSLATION_VELOCITY = { 150, 150, 150 };
     private ISmartServoLINRuntime _smartServoLINRuntime = null;
	 private static final int NUM_RUNS = 600;
	 private static final double AMPLITUDE = 70;
	 private SafeDataIOGroup SafeDataIO;
	 private JointPosition jointPos;
	 private JointPosition jointPos_zuo;
	 private JointPosition jointPos_you;
	 public boolean bForPlane;
	 Frame pre_Place;
	 
	 int  num_ForTest=0;
	 int count=0;
//	@Named("gripper")
//	@Inject
//	private Tool gripper;
	
//	@Named("Tool_2")
//	@Inject
//	private Tool Tool_2;
	
	@Named("Tool_2")
	
	@Inject
	private Tool needle;
	
	@Named("gripper")
	@Inject
	private Tool needle_gripper;
	
	private ObjectFrame tcp;
	private ObjectFrame tcp_2;
	private ObjectFrame handle;
	private BooleanIOCondition VaccumDetect;
	private Frame startpos;
	private Tool tool;
	public boolean bDangerous=false;
	
	//å�‘é€�å­—ç¬¦ä¸² data0~data18
	public static String data0="0";
	public static String data1="0";
	public static String data2="0";
	public static String data3="0";
	public static String data4="0";
	public static String data5="0";
	public static String data6="0";
	public static String data7="0";
	public static String data8="0";
	public static String data9="0";
	public static String data10="0";
	public static String data11="0";
	public static String data12="0";
	public static String data13="0";
	public static String data14="0";
	public static String data15="0";
	public static String data16="0";
	public static String data17="0";
	public static String data18="0";
    
    public double nintegral=0;
    public double nderivative=0;
    public double nPrevious_error=0;
    public double nOutput=0;
    public double nP=0.5;
    public double nI=0.02;
    public double nD=0.01;
    public double nDistance=0;
    public double nStiff=0;
    
	//å…¨å±€X,Y,Zå�˜é‡� è¾“å…¥å�˜é‡�
	public static double nX=0;
	public static double nY=0;
	public static double nZ=0;
	public static double nA=0;
	public static double nB=0;
	public static double nC=0;
	public String Err="0,";
	
	 JointPosition currentPos_test;

	//å…¨å±€å·¥ä½œæ¨¡å¼�å�˜é‡� è¾“å…¥å�˜é‡�
	public static int nWorkingmode=0;
	public static int nToolMode=0;
	@Inject
	private CopyOfTeachingByHand_2 JointImpedanceMode;
	@Override
	public void initialize() {
		count=0;
		Err="0,";
		num_ForTest=0;
		OnlyPlane.initialize();
		bForPlane=false;
		nWorkingmode=0;
		nToolMode=1;
		kuka_Sunrise_Cabinet_1 = getController("KUKA_Sunrise_Cabinet_1");
		SafeDataIO = new SafeDataIOGroup(kuka_Sunrise_Cabinet_1);
		nX=0;
		nY=0;
		nZ=0;
		nA=0;
		nB=0;
		nC=0;
		io.setOutput5(false);
		jointPos=new JointPosition(   Math.toRadians(0),
                Math.toRadians(28.28),
                Math.toRadians(22.32),
                Math.toRadians(85.38),
                Math.toRadians(-0.56),
                Math.toRadians(-59.95),
                Math.toRadians(27.77));
		
		jointPos_zuo=new JointPosition(   Math.toRadians(26.39),
                Math.toRadians(20.06),
                Math.toRadians(-15.64),
                Math.toRadians(77.97),
                Math.toRadians(-98.35),
                Math.toRadians(32.88),
                Math.toRadians(-30.42));
		
		jointPos_you=new JointPosition(   Math.toRadians(-9.91),
                Math.toRadians(-4.63),
                Math.toRadians(-44.98),
                Math.toRadians(115),
                Math.toRadians(-2.29),
                Math.toRadians(-65.18),
                Math.toRadians(-127));
		

		
		
		try {
			ThreadUtil.milliSleep(1000);
			if(serverSocket!=null){
				serverSocket.close();
				serverSocket=null;
				System.out.println("022");
			}
			if(serverSocketSend!=null ){
				serverSocketSend.close();
				serverSocketSend=null;
				System.out.println("02222");
			}
			if(writer!=null ){
				writer.close();
				writer=null;
				System.out.println("0333");
			}
			if(outputStream!=null ){
				outputStream.close();
				outputStream=null;
				System.out.println("033333");
			}
			if(socket!=null ){
				socket.close();
				socket=null;
				System.out.println("033333");
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		
       lbr = getContext().getDeviceFromType(LBR.class);

     	_loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofRad(TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1], TRANSLATION_OF_TOOL[2],0,-1.047,0);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(lbr.getFlange());
        
		needle.attachTo(lbr.getFlange());
		needle_gripper.attachTo(lbr.getFlange());
	}

	public  class sendRTdata implements Callable<String> {
         
		 public void GetData()
		    {
//		        ThreadUtil.milliSleep(1000);
				//æš‚æ—¶æ— æ„�ä¹‰ï¼ˆé¢„ç•™é»˜è®¤ä¸º0ï¼‰
				data0 = "$0,";
				
				//æŠ¥è­¦ä»£ç �
				data1 = "0,";
				
				//å·¥ä½œæ¨¡å¼�
		
				data2 = Err;
//				System.out.println(Err);
				//æ˜¯å�¦ä¸Šç”µ
				data3 = "2,";
				
				//å½“å‰�é€Ÿåº¦
				//CartesianVelocityLimitInfo infoObject = lbr.getCartesianVelocityLimitInfo();
				double a1 =getApplicationControl().getApplicationOverride();
				BigDecimal bigDecimal0 = new BigDecimal(a1);
				a1 = bigDecimal0.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data4 = String.valueOf(a1)+",";
				
				//æ— æ„�ä¹‰
				data5 = "3,";
				
//				//å…³èŠ‚åº¦æ•°1
				JointPosition actPos = lbr.getCurrentJointPosition();
				a1 = Math.toDegrees(actPos.get(0));
				BigDecimal bigDecimal = new BigDecimal(a1);
				a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data6=String.valueOf(a1)+",";
				
//				//å…³èŠ‚åº¦æ•°2
				a1 = Math.toDegrees(actPos.get(1));
				BigDecimal bigDecimal1 = new BigDecimal(a1);
				a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data7=String.valueOf(a1)+",";
				
//				//å…³èŠ‚åº¦æ•°3
				a1 = Math.toDegrees(actPos.get(2));
				BigDecimal bigDecimal2 = new BigDecimal(a1);
				a1 = bigDecimal2.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data8=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°4
				a1 = Math.toDegrees(actPos.get(3));
				BigDecimal bigDecimal3 = new BigDecimal(a1);
				a1 = bigDecimal3.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data9=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°5
				a1 = Math.toDegrees(actPos.get(4));
				BigDecimal bigDecimal4 = new BigDecimal(a1);
				a1 = bigDecimal4.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data10=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°6
				a1 = Math.toDegrees(actPos.get(5));
				BigDecimal bigDecimal5 = new BigDecimal(a1);
				a1 = bigDecimal5.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data11=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°7
				a1 = Math.toDegrees(actPos.get(6));
				BigDecimal bigDecimal6 = new BigDecimal(a1);
				a1 = bigDecimal6.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data12=String.valueOf(a1)+",";
				
				//è½´å��æ ‡x
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
				
				
				
//				if (nToolMode==2){
//					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_anfang"));
//				}
//				else{
//					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//				}
				
				if (nToolMode==1){
					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
						cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//						System.out.println("1:"+cmdPos);
						Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
						cmdPos2.setX(0);
						cmdPos2.setY(0);
						cmdPos2.setZ(0);
						cmdPos2.setAlphaRad(0);
						cmdPos2.setBetaRad(Math.toRadians(-30));
						cmdPos2.setGammaRad(0);
						cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"), cmdPos2);
						//System.out.println("22:"+cmdPos);
						
				}
				else if(nToolMode==2)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
//					System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"), cmdPos2);
//					System.out.println("2:"+cmdPos);
				}	
				else if(nToolMode==3)
				{				
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz2"));
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz2"));
//					System.out.println("1:"+cmdPos);
					Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz2"), cmdPos2);
					
				}	
				else if(nToolMode==4)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
				}	
				else if(nToolMode==5)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
				}	
				else if(nToolMode==6)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz2"));
				}	
				else if(nToolMode==7)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz3"));
				}	
				else if(nToolMode==8)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz4"));
				}	
				else if(nToolMode==9)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_3_yz3"));
				}	
				else if(nToolMode==10)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_3_yz4"));
				}	
				else
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/test"));
				}
				
				a1=cmdPos.getX();
				BigDecimal bigDecimal7 = new BigDecimal(a1);
				a1 = bigDecimal7.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data13=a1+",";
				data13=String.valueOf(a1)+",";
				
				//è½´å��æ ‡y
				a1=cmdPos.getY();
				BigDecimal bigDecimal8 = new BigDecimal(a1);
				a1 = bigDecimal8.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data14=a1+",";
				data14=String.valueOf(a1)+",";
				
				//è½´å��æ ‡z
				a1=cmdPos.getZ();
				BigDecimal bigDecimal9 = new BigDecimal(a1);
				a1 = bigDecimal9.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data15=a1+",";
				data15=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rx
				a1=Math.toDegrees(cmdPos.getAlphaRad());
				BigDecimal bigDecima20 = new BigDecimal(a1);
				a1 = bigDecima20.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data16=a1+",";
				data16=String.valueOf(a1)+",";
				
				//è½´å��æ ‡ry
				a1=Math.toDegrees(cmdPos.getBetaRad());
				BigDecimal bigDecima21 = new BigDecimal(a1);
				a1 = bigDecima21.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data17=a1+",";
				data17=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rz

				
				a1=Math.toDegrees(cmdPos.getGammaRad());
				BigDecimal bigDecima22 = new BigDecimal(a1);
				a1 = bigDecima22.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data18=a1+"$";
				data18=String.valueOf(a1)+",$";
		    }
		
		public String call() {
			
			
			
			
			while(true){
			try {
			boolean bPause=false;
			serverSocketSend = new ServerSocket(30001);
			
			
			
			System.out.println("New socket.");
			socket= serverSocketSend.accept();
//			socket.setSoTimeout(2500);
			System.out.println("Socket accepted. IP:{" + socket.getInetAddress().getHostAddress() + "}.");
			outputStream = new DataOutputStream(socket.getOutputStream());
			writer= new OutputStreamWriter(outputStream);

			while (bPause==false)
			{


				try{
					GetData();
//					ThreadUtil.milliSleep(1000);
//					System.out.println("x1");
////				    System.out.println(socket_recive.isBound());
////				    System.out.println(socket_recive.isConnected());
//				    System.out.println("x2");
////				    System.out.println(socket_recive.isClosed());
//				    System.out.println("x3");
//				    System.out.println(socket_recive.getOOBInline());
				    
					String data=data0+data1+data2+data3+data4+data5+data6+data7+data8+data9+data10+data11+data12+data13+data14+data15+data16+data17+data18;

			
					writer.write(data);
					writer.flush();
					ThreadUtil.milliSleep(50);
//					ThreadUtil.milliSleep(2000);
					}
				catch (IOException e) {
					System.out.println("Socket closed.");					
					System.out.println("Closing socket.");
					
					writer.close();
					outputStream.close();
					socket.close();
					serverSocketSend.close();
					writer=null;
					outputStream=null;
					socket=null;
					serverSocketSend=null;
					bPause=true;
					
//					try {
//						ThreadUtil.milliSleep(1000);
//						if(serverSocket!=null){
//							serverSocket.close();
//							serverSocket=null;
//							System.out.println("222");
//						}
//						if(serverSocketSend!=null ){
//							serverSocketSend.close();
//							serverSocketSend=null;
//							System.out.println("22222");
//						}
//						if(writer!=null ){
//							writer.close();
//							writer=null;
//							System.out.println("3333");
//						}
//						if(outputStream!=null ){
//							outputStream.close();
//							outputStream=null;
//							System.out.println("133333");
//						}
//						if(socket!=null ){
//							socket.close();
//							socket=null;
//							System.out.println("233333");
//						}
//					} catch (IOException f) {
//						// TODO Auto-generated catch block
//						e.printStackTrace();
//					}
					
				}
				

			}
			
			System.out.println("closed.");					
			System.out.println("Socket.");
			

		} catch (IOException e) {
			e.printStackTrace();
		};
		}
		}
	}

	public  class reciveRTdata implements Callable<String> {
        
        
		
		public String call() {	
			String[] units=null;
			while(true){
			try{
			boolean bPause=false;
			serverSocket = new ServerSocket(30007);
			
			System.out.println("New socket.");
		    socket_recive = serverSocket.accept();
		   // socket_recive.setSoTimeout(2500);
			System.out.println("Socket accepted. IP:{" + socket_recive.getInetAddress().getHostAddress() + "}.");
		    
			InputStream in = socket_recive.getInputStream();
//			outputStream_recive
			outputStream_recive = new DataOutputStream(socket_recive.getOutputStream());
			writer_recive= new OutputStreamWriter(outputStream_recive);
//			PrintWriter writer = new PrintWriter(new OutputStreamWriter(socket.getOutputStream()));
			
			while(bPause==false){

				try{
					
//					byte[] receivingBuffer = new byte[256];
//					in.read(receivingBuffer);
					
				      byte[] buf = new byte[1];
				        int size = 0;
//				        System.out.println("qq");
				        StringBuffer sb = new StringBuffer();
				    
//				        System.out.println(in.read(buf,0,buf.length));
				        if (in.read(buf,0,buf.length)==-1)
				        {
				        	bPause=true;
//				        	System.out.println("oo");
				        	break;
				        }
//				        System.out.println("ee");
				        while (( size = in.read(buf,0,buf.length)) != -1) {
//				            System.out.println("ww");
				            String str = new String(buf);

				            sb.append(str);
//				            System.out.print(str);
//				            System.out.println(str);
				            if(str.equals("*")) {
//								System.out.println(new String(sb).trim());
//								System.out.println("tt");
								units = new String(sb).trim().split(",");
								
				                break;
				            }
				        }
//				        System.out.println("rr");
				        

					if(units[1] == null){
						System.out.println("data receiving error: no command received.....");
					}else{
						if(units[1].equals("spd")){
//						
							String para = units[2].substring(0, units[2].length() - 1);
//							System.out.println("spd: " + para);
							writer_recive.write("$res,spd,1");
							writer_recive.flush();
							
						}
						else if(units[1].equals("mp")){
							System.out.println("para: " + units[2]);
							System.out.println("para: " + units[3]);
							System.out.println("para: " + units[4]);
							System.out.println("para: " + units[5]);
							System.out.println("para: " + units[6]);
							String para1 = units[7].substring(0, units[7].length() - 1);
							System.out.println("para" + para1);

							nX=Double.parseDouble(units[2]);
							nY=Double.parseDouble(units[3]);
							nZ=Double.parseDouble(units[4]);
							nA=Double.parseDouble(units[5]);
							nB=Double.parseDouble(units[6]);
							nC=Double.parseDouble(para1);
							
							//ss

							
							pre_Place = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
							pre_Place.setX(nX);
							pre_Place.setY(nY);
							pre_Place.setZ(nZ);
							pre_Place.setAlphaRad(Math.toRadians(nA));
							pre_Place.setBetaRad(Math.toRadians(nB));
							pre_Place.setGammaRad(Math.toRadians(nC));
							System.out.println("pre_Place"+pre_Place);
							System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
							writer_recive.write("$para,mp,0$");
							writer_recive.flush();
						}
						else if(units[1].equals("sIO")){
//							System.out.println("para: " + units[2]);
							String para2 = units[3].substring(0, units[3].length() - 1);
							if(Double.parseDouble(para2)==1 && Double.parseDouble(units[2])==1)
							{
							io.setOutput5(true);	
//							System.out.println("io.setOutput5(true)");
							}
							else{
							io.setOutput5(false);
//							System.out.println("io.setOutput5(false)");
//							System.out.println("*"+Double.parseDouble(para2)+"*  "+"*"+Double.parseDouble(units[2])+"*");
							}
//							System.out.println("sIO: " + para2);
							writer_recive.write("$res,sIO,0$");
							writer_recive.flush();
							
						}
						else if(units[1].equals("RobotMove")){
//							System.out.println("para: " + units[2]);
							String para2 = units[3].substring(0, units[3].length() - 1);
							if(Double.parseDouble(para2)==2 )
							{
								System.out.println("RobotMove2");
								nToolMode=2;
							}
							else if(Double.parseDouble(para2)==1 ) {
								System.out.println("RobotMove1");
								nToolMode=1;
							}
							else if(Double.parseDouble(para2)==3 ) {
								System.out.println("RobotMove3");
								nToolMode=3;
							}
							else if(Double.parseDouble(para2)==4 ) {
								System.out.println("RobotMove4");
								nToolMode=4;
							}
							else if(Double.parseDouble(para2)==5 ) {
								System.out.println("RobotMove5");
								nToolMode=5;
							}
							else if(Double.parseDouble(para2)==6 ) {
								System.out.println("RobotMove6");
								nToolMode=6;
							}
							else if(Double.parseDouble(para2)==7 ) {
								System.out.println("RobotMove7");
								nToolMode=7;
							}
							else if(Double.parseDouble(para2)==8 ) {
								System.out.println("RobotMove8");
								nToolMode=8;
							}
							else if(Double.parseDouble(para2)==9 ) {
								System.out.println("RobotMove9");
								nToolMode=9;
							}
							else if(Double.parseDouble(para2)==10 ) {
								System.out.println("RobotMove10");
								nToolMode=10;
							}
							writer_recive.write("$res,RobotMove,0$");
							writer_recive.flush();
							
						}
						else if(units[1].equals("setwm")){
							String para3 = units[2].substring(0, units[2].length() - 1);
//							System.out.println("setwm: " + para3);
							nWorkingmode=Integer.parseInt(para3);
							writer_recive.write("$res,setwm"+nWorkingmode);
							System.out.println("setwm:"+nWorkingmode);
							writer_recive.flush();
							
						}
						else if(units[1].equals("stcp")){
							System.out.println("normal " + units);
							String para4 = units[2].substring(0, units[2].length() - 1);
							System.out.println("stcp: " + para4);
//							nToolMode=Integer.parseInt(para4);
							writer_recive.write("$res,stcp,1");
							writer_recive.flush();
						}
						else{
							writer_recive.write("$res,err,-1");
							writer_recive.flush();
						}
					
					}
					ThreadUtil.milliSleep(20);
					
				}catch (IOException e) {
					System.out.println("closed.");					
					System.out.println("Socket.");
					
					in.close();
					writer_recive.close();
					outputStream_recive.close();
					socket_recive.close();
					serverSocket.close();
					
					writer_recive=null;
					outputStream_recive=null;
					socket_recive=null;
					serverSocket=null;
					in=null;
					bPause=true;
					
					
				}
				
//				System.out.println("closed...");					
//				System.out.println("Socket...");

			}
			System.out.println("closed11.");					
			System.out.println("Socket11.");
			
			if(in!=null){
				in.close();
				in=null;
				System.out.println("in");
			}
			if(writer_recive!=null){
				writer_recive.close();
				writer_recive=null;
				System.out.println("writer_recive");
			}
			if(outputStream_recive!=null ){
				outputStream_recive.close();
				outputStream_recive=null;
				System.out.println("outputStream_recive");
			}
			if(socket_recive!=null ){
				socket_recive.close();
				socket_recive=null;
				System.out.println("socket_recive");
			}
			if(serverSocket!=null ){
				serverSocket.close();
				serverSocket=null;
				System.out.println("serverSocket");
			}
			
			
			
//			in.close();
//			writer_recive.close();
//			outputStream_recive.close();
//			socket_recive.close();
//			serverSocket.close();
//			
//			writer_recive=null;
//			outputStream_recive=null;
//			socket_recive=null;
//			serverSocket=null;
//			in=null;
			bPause=true;
			
			System.out.println("closed000");					
			System.out.println("Socket000");
			}catch(IOException e){
//				System.out.println("closed//...");		
				
				System.out.println("Socket//IOException e");
				ThreadUtil.milliSleep(1000);
//				System.out.println("closed.");					
//				System.out.println("Socket.");
//				in.close();
//				writer.close();
//				socket.close();
//				serverSocket.close();
//				
//				in=null;
//				writer=null;
//				//			outputStream=null;
//				socket=null;
//				serverSocket=null;
			};
//			return "end";
			}
		}
	}

	
public HandGuidingMotion createhandGuidingMotion(){
		
	double min1=0;
	double max1=0;
	JointPosition jReady =lbr.getCurrentJointPosition();
	if(Math.toDegrees(jReady.get(JointEnum.J1)) < -40){
		min1=Math.toDegrees(jReady.get(JointEnum.J1));
	}
	else
	{
		min1=-40;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J1)) > 40){
		max1=Math.toDegrees(jReady.get(JointEnum.J1));
	}
	else
	{
		max1=40;
	}
	
	double min2=0;
	double max2=0;

	if(Math.toDegrees(jReady.get(JointEnum.J2)) < -60){
		min2=Math.toDegrees(jReady.get(JointEnum.J2));
	}
	else
	{
		min2=-60;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J2)) > 60){
		max2=Math.toDegrees(jReady.get(JointEnum.J2));
	}
	else
	{
		max2=60;
	}
	
	
	double min3=0;
	double max3=0;

	if(Math.toDegrees(jReady.get(JointEnum.J3)) < -55){
		min3=Math.toDegrees(jReady.get(JointEnum.J3));
	}
	else
	{
		min3=-55;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J3)) > 10){
		max3=Math.toDegrees(jReady.get(JointEnum.J3));
	}
	else
	{
		max3=10;
	}
	
	double min4=0;
	double max4=0;

	if(Math.toDegrees(jReady.get(JointEnum.J4)) < 50){
		min4=Math.toDegrees(jReady.get(JointEnum.J4));
	}
	else
	{
		min4=50;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J4)) > 100){
		max4=Math.toDegrees(jReady.get(JointEnum.J4));
	}
	else
	{
		max4=100;
	}
	
	double min5=0;
	double max5=0;

	if(Math.toDegrees(jReady.get(JointEnum.J5)) < -150){
		min5=Math.toDegrees(jReady.get(JointEnum.J5));
	}
	else
	{
		min5=-150;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J5)) > 50){
		max5=Math.toDegrees(jReady.get(JointEnum.J5));
	}
	else
	{
		max5=50;
	}
	
	double min6=0;
	double max6=0;

	if(Math.toDegrees(jReady.get(JointEnum.J6)) < 0){
		min6=Math.toDegrees(jReady.get(JointEnum.J6));
	}
	else
	{
		min6=0;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J6)) > 100){
		max6=Math.toDegrees(jReady.get(JointEnum.J6));
	}
	else
	{
		max6=100;
	}
	
	double min7=0;
	double max7=0;

	if(Math.toDegrees(jReady.get(JointEnum.J7)) < -150){
		min7=Math.toDegrees(jReady.get(JointEnum.J7));
	}
	else
	{
		min7=-150;
	}
	
	if(Math.toDegrees(jReady.get(JointEnum.J7)) > 150){
		max7=Math.toDegrees(jReady.get(JointEnum.J7));
	}
	else
	{
		max7=150;
	}
	
	
		HandGuidingMotion motion = new HandGuidingMotion();
		motion.setJointVelocityLimit(0.8)
		.setCartVelocityLimit(900.0).setJointLimitViolationFreezesAll(false)
		.setJointLimitsMax(Math.toRadians(max1), Math.toRadians(max2), Math.toRadians(max3), Math.toRadians(max4), Math.toRadians(max5),Math.toRadians(max6), Math.toRadians(max7))
		.setJointLimitsMin(Math.toRadians(min1), Math.toRadians(min2), Math.toRadians(min3), Math.toRadians(min4), Math.toRadians(min5),Math.toRadians(min6), Math.toRadians(min7))
		.setJointLimitsEnabled(true,true,true,true,true,true,true)


		

;
		return motion;
	}

	public CartesianImpedanceControlMode createCartImp(){
		CartesianImpedanceControlMode mode = new CartesianImpedanceControlMode();
		mode.parametrize(CartDOF.ROT).setStiffness(300);
		mode.parametrize(CartDOF.X).setStiffness(1);
		mode.parametrize(CartDOF.Y).setStiffness(5000);
		mode.parametrize(CartDOF.Z).setStiffness(5000);
	
		return mode;
	}
	

	
	
	public  class motion implements Callable<String> {
		
		
	    public void moveToInitialPosition()
	    {
	    	System.out.println("11");
	        if (!ServoMotion.validateForImpedanceMode(lbr))
	        {
	            getLogger()
	                    .info("Validation of torque model failed - correct your mass property settings");
	            getLogger()
	                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
	        }
	    }
	    
		//è½´é˜»æŠ—æŽ§åˆ¶ä»£ç �
	    public void runSmartServoMotion(final IMotionControlMode controlMode)
	    {

	    	System.out.println("22");
	        final boolean doDebugPrints = false;

	        final JointPosition initialPosition = new JointPosition(
	                lbr.getCurrentJointPosition());
	        System.out.println("33");
	        final SmartServo aSmartServoMotion = new SmartServo(initialPosition);

	        // Set the motion properties to 10% of the systems abilities
	        aSmartServoMotion.setJointAccelerationRel(0.1);
	        aSmartServoMotion.setJointVelocityRel(0.1);
	        System.out.println("44");
	        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

	        getLogger().info("Starting the SmartServo in " + controlMode);
	        IMotionContainer container = _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
	                aSmartServoMotion.setMode(controlMode));
	        // Fetch the Runtime of the Motion part
	        final ISmartServoRuntime theSmartServoRuntime = aSmartServoMotion
	                .getRuntime();

	        // create an JointPosition Instance, to play with
	        final JointPosition destination = new JointPosition(
	                lbr.getJointCount());
	        System.out.println("55");
	        // For Roundtrip time measurement...
	        final StatisticTimer timing = new StatisticTimer();
	        try
	        {
	            // do a cyclic loop
	            // Refer to some timing...
	            // in nanosec
	            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
	            final long startTimeStamp = System.nanoTime();
                final JointPosition currentPos = lbr.getCurrentJointPosition();

	            while(SafeDataIO.getInput4()==false)
	            {
	            	final JointPosition currentPos_now = lbr.getCurrentJointPosition();
//	            	System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));
	           
	                // Timing - draw one step
	                final OneTimeStep aStep = timing.newTimeStep();
	                // ///////////////////////////////////////////////////////
	                // Insert your code here
	                // e.g Visual Servoing or the like
	                // emulate some computational effort - or waiting for external
	                // stuff
	                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

	                theSmartServoRuntime.updateWithRealtimeSystem();

	                // Get the measured position in cartesian...
	                final JointPosition curMsrJntPose = theSmartServoRuntime
	                        .getAxisQMsrOnController();

	                
	   
	                if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J5))) > 160 || Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J6))) > 110 || Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J7))) > 165){
	                	 if (controlMode instanceof JointImpedanceControlMode)
	                       {  		 
	                		 
	                       final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
	                       double para =200;
	                       double para1=200;
	                       double para2=200;
	                       
	                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J5))) > 160){
	                    	   para = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5))));  
//	                    	   System.out.println(para);
	                       }
	                       else{
	                    	   para=200;
	                       }
	                       
	                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J6))) > 110){
	                    	   para1 = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(120)) - Math.abs(currentPos_now.get(JointEnum.J6))));  
	                       }
	                       else{
	                    	   para1=200;
	                       }
	                       
	                       if(Math.abs(Math.toDegrees(currentPos_now.get(JointEnum.J7))) > 165){
	                    	   para2 = 2000 / Math.toDegrees(Math.abs(Math.abs(Math.toRadians(175)) - Math.abs(currentPos_now.get(JointEnum.J7))));  
	                       }
	                       else{
	                    	   para2=200;
	                       }
	                       
//	                       System.out.println("xishu "+Math.toDegrees(Math.abs(Math.abs(Math.toRadians(170)) - Math.abs(currentPos_now.get(JointEnum.J5)))));
	                       jointImp.setStiffness(2000, 2000, 2000, 2000, para, para1, para2);
//	                       System.out.println(para);
	                       jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
	                       theSmartServoRuntime
	                               .changeControlModeSettings(jointImp);
	                		 
	                		 
	                       }
	                }
	                else{
	                
	                    
	                	if (controlMode instanceof JointImpedanceControlMode)
	               	 	{
	               	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
	               	 		jointImp.setStiffness(2000, 2000, 2000, 2000, 200, 200, 200);
	               	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
	               	 		// Send the new Stiffness settings down to the
	               	 		// controller
	               	 		theSmartServoRuntime
	               	 		.changeControlModeSettings(jointImp);
	               	 	} 	
	                }
	                
	                
//	                
//	                if(Math.toDegrees(currentPos_now.get(JointEnum.J5))>160)
//	                {
//	                JointPosition currentPos1=new JointPosition(currentPos.get(JointEnum.J1), currentPos.get(JointEnum.J2), currentPos.get(JointEnum.J3), currentPos.get(JointEnum.J4), 2.7925, currentPos.get(JointEnum.J6), currentPos.get(JointEnum.J7));
//	                destination.set(currentPos1);
//	            	System.out.println("change.."+currentPos1);
//	                }
//	                else{
//	                	destination.set(currentPos);
//	                }
	                
	                destination.set(currentPos);
	                
	                theSmartServoRuntime
	                        .setDestination(destination);

	                aStep.end();


	            }
	        }
	        catch (final Exception e)
	        {
	            getLogger().info(e.getLocalizedMessage());
	            e.printStackTrace();
	        }

	        //Print statistics and parameters of the motion
	        getLogger().info("Displaying final states after loop "
	                + controlMode.getClass().getName());
	        getLogger().info(getClass().getName() + theSmartServoRuntime.toString());
	        // Stop the motion
	        theSmartServoRuntime.stopMotion();
	        getLogger().info("Statistic Timing of Overall Loop " + timing);
	        if (timing.getMeanTimeMillis() > 150)
	        {
	            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
	            getLogger().info("Under Windows, you should play with the registry, see the e.g. the RealtimePTP Class javaDoc for details");
	        }

	    }
		 
		    protected JointImpedanceControlMode createJointImp()
		    {
		        final JointImpedanceControlMode jointImp = new JointImpedanceControlMode(2000, 2000, 2000, 2000, 200, 200, 200);
//		        jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
		   
		        jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
		        return jointImp;
		    }
		    
		    protected CartesianImpedanceControlMode ConeLimit()
		    {
		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		        cartImp.parametrize(CartDOF.X).setStiffness(500.0);
		        cartImp.parametrize(CartDOF.Y).setStiffness(500.0);
		        cartImp.parametrize(CartDOF.Z).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);
	
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		   
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
		        cartImp.setMaxPathDeviation(150., 150., 150., 3., 3., 3.);
		        return cartImp; 	
		    }

		    protected CartesianImpedanceControlMode HardLimit()
		    {

		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		
		        cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.Y).setStiffness(150.0);
		        cartImp.parametrize(CartDOF.Z).setStiffness(150.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);
		        cartImp.parametrize(CartDOF.C).setStiffness(20.0);
//		        System.out.println(nStiff);
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		   
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
		        cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
		        return cartImp; 	
		    }
		    
		    protected CartesianImpedanceControlMode HardLimit_Y()
		    {

		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		
		        cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.Y).setStiffness(4500.0);
		        cartImp.parametrize(CartDOF.Z).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

//		        System.out.println(nStiff);
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		   
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
		        cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
		        return cartImp; 	
		    }
		    
		    protected CartesianImpedanceControlMode HardLimit_Z()
		    {

		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		
		        cartImp.parametrize(CartDOF.X).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.Y).setStiffness(5000.0);
		        cartImp.parametrize(CartDOF.Z).setStiffness(4500.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(300.0);

//		        System.out.println(nStiff);
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		   
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(100.0, 100.0, 50.0, 20.0, 20.0, 20.0, true);
//		        cartImp.setMaxControlForce(1, 1, 1, 1, 1, 1, true);
		        cartImp.setMaxPathDeviation(1500., 1500., 1500., 3., 3., 3.);
		        return cartImp; 	
		    }
		    
		    protected CartesianImpedanceControlMode createCartImp()
		    {
		        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
		        cartImp.parametrize(CartDOF.TRANSL).setStiffness(1000.0);
		        cartImp.parametrize(CartDOF.ROT).setStiffness(100.0);
//		        cartImp.parametrize(CartDOF.X).setAdditionalControlForce(-4.9);
		        cartImp.setNullSpaceStiffness(100.);
		        
//		        cartImp.setMaxCartesianVelocity(5.0,5.0,5.0,0.2,0.2, 0.2);
		        // For your own safety, shrink the motion abilities to useful limits
		        cartImp.setMaxPathDeviation(150., 150., 50., 50., 50., 50.);
		        return cartImp;
		    }
		    
		 //ç¬›å�¡å°”é˜»æŠ—æŽ§åˆ¶ä»£ç �
		    public void runSmartCartesianMotion(final IMotionControlMode controlMode){
//		     	_loadData = new LoadData();
//		        _loadData.setMass(MASS);
//		        _loadData.setCenterOfMass(
//		                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
//		                CENTER_OF_MASS_IN_MILLIMETER[2]);
//		        _toolAttachedToLBR = new Tool("Tool", _loadData);
//
//		        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
//		                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
//		                TRANSLATION_OF_TOOL[2]);
//		        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
//		                + "(TCP)", trans);
//		        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
//		        // Attach tool to the robot
//		        _toolAttachedToLBR.attachTo(lbr.getFlange());
		        
		        final JointPosition initialPosition = new JointPosition(
		                lbr.getCurrentJointPosition());

		        final DirectServo aDirectServoMotion = new DirectServo(initialPosition);

		        aDirectServoMotion.setMinimumTrajectoryExecutionTime(40e-3);

		        getLogger().info("Starting the DirectServo motion in " + controlMode);
		        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
		                aDirectServoMotion.setMode(controlMode));
		        System.out.println("sss ");
		        final IDirectServoRuntime theServoRuntime = aDirectServoMotion
		                .getRuntime();
		        System.out.println("ggg ");
		        // create an JointPosition Instance, to play with
		        final JointPosition destination = new JointPosition(
		                lbr.getJointCount());
		        System.out.println("vvv ");
		        // For Roundtrip time measurement...
		        final StatisticTimer timing = new StatisticTimer();
		        System.out.println("ccc");
		        try
		        {
		            // do a cyclic loop
		            // Refer to some timing...
		            // in nanosec
		            // freqency * ...
		            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
		            final long startTimeStamp = System.nanoTime();

		            //for (int i = 0; i < NUM_RUNS; ++i)
		         	final JointPosition currentPos = lbr.getCurrentJointPosition();
		         	
		         	JointPosition currentPos_CheckSafety;
//		         	destination.set(currentPos_test);
		         	destination.set(currentPos);
		         	
		         	boolean bDanger=false;
//		         	System.out.println("hhh");
		         	System.out.println(bDangerous+"dd"+nWorkingmode);
		         	
		            while (nWorkingmode==4 && bDangerous==false)
		            {
		          
		            	currentPos_CheckSafety=lbr.getCurrentJointPosition();
		            	if (Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J1))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J2))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J3))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J4))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J5))) < 160 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J6))) < 110 && Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J7))) < 165){
		            		bDangerous=false;
		            		
		            	}
		            	else{
		            		System.out.println("J1"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J1)));
		            		System.out.println("J2"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J2)));
		            		System.out.println("J3"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J3)));
		            		System.out.println("J4"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J4)));
		            		System.out.println("J5"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J5)));
		            		System.out.println("J6"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J6)));
		            		System.out.println("J7"+Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J7)));
		            		getLogger().info("bDangerous==true");
		            		
		            		bDangerous=true;
		            	}
		            	
		                theServoRuntime.setDestination(destination);
		                     
//		                // Timing - draw one step
//		                final OneTimeStep aStep = timing.newTimeStep();
//		                // ///////////////////////////////////////////////////////
//		                // Insert your code here
//		                // e.g Visual Servoing or the like
//		                // emulate some computational effort - or waiting for external
//		                // stuff
//		                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
		//
//		                theServoRuntime.updateWithRealtimeSystem();
		//
//		                final double curTime = System.nanoTime() - startTimeStamp;
//		                final double sinArgument = omega * curTime;
		//
//		                for (int k = 0; k < destination.getAxisCount(); ++k)
//		                {
//		                    destination.set(k, Math.sin(sinArgument)
//		                            * AMPLITUDE + initialPosition.get(k));
//		                }
//		                theServoRuntime
//		                        .setDestination(destination);
		//
//		                // Modify the stiffness settings every now and then               
//		                if (i % (NUM_RUNS / 10) == 0)
//		                {
//		                    // update realtime system
//		                    if (controlMode instanceof CartesianImpedanceControlMode)
//		                    {
//		                        final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
//		                        final double aTransStiffVal = Math.max(100. * (i
//		                                / (double) NUM_RUNS + 1), 1000.);
//		                        final double aRotStiffVal = Math.max(10. * (i
//		                                / (double) NUM_RUNS + 1), 150.);
//		                        cartImp.parametrize(CartDOF.TRANSL).setStiffness(aTransStiffVal);
//		                        cartImp.parametrize(CartDOF.ROT).setStiffness(aRotStiffVal);
//		                        // Send the new Stiffness settings down to the
//		                        // controller
//		                        theServoRuntime
//		                                .changeControlModeSettings(cartImp);
//		                    }
//		                }
//		                aStep.end();
		            }
		        }
		        catch (final Exception e)
		        {
		            getLogger().info(e.getLocalizedMessage());
		            e.printStackTrace();
		        }

		        // Print statistics and parameters of the motion
		        getLogger().info("Displaying final states after loop "
		                + controlMode.getClass().getName());

		        getLogger().info(getClass().getName() + "\n" + theServoRuntime.toString());

		        // Stop the motion
		        theServoRuntime.stopMotion();
		        getLogger().info("Statistic Timing of Overall Loop " + timing);
		        if (timing.getMeanTimeMillis() > 150)
		        {
		            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
		            getLogger().info("You should check the TCP/IP Stack Configuration - see the manual for details");
		        }
		    }
	
		    private StatisticTimer startSineMovement(
		            ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
		    {
		        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(lbr
		                .getFlange());

		        getLogger().info("Do sine movement");
		        try
		        {
		            double omega = FREQENCY * 2 * Math.PI * 1e-9;
		            long startTimeStamp = System.nanoTime();
		        	Frame Ptest2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
		         	
		           while(nWorkingmode==2)
		            {
		            
		                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
							//$cmd,RobotMove,1$
							if(nX!=0 && nY!=0 && nZ!=0 ){
								Ptest2.setX(nX);
								Ptest2.setY(nY);
								Ptest2.setZ(nZ);
//								System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
								Ptest2.setAlphaRad(Math.toRadians(nA));
								Ptest2.setBetaRad(Math.toRadians(nB));
								Ptest2.setGammaRad(Math.toRadians(nC));
								
//								ThreadUtil.milliSleep(500);
//								System.out.println("222");
							}
							
		                final OneTimeStep aStep = timing.newTimeStep();
		                // ///////////////////////////////////////////////////////
		                // Insert your code here
		                // e.g Visual Servoing or the like
		                // Synchronize with the realtime system

		                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

		                // Update the smart servo LIN runtime
		                smartServoLINRuntime.updateWithRealtimeSystem();

		                double curTime = System.nanoTime() - startTimeStamp;
		                double sinArgument = omega * curTime;

		                // Compute the sine function
		                Frame destFrame = new Frame(aFrame);
		                destFrame.setZ(AMPLITUDE * Math.sin(sinArgument));

		                // Set new destination
		                smartServoLINRuntime.setDestination(Ptest2);
		                aStep.end();
		            }

		        }
		        catch (Exception e)
		        {
		            getLogger().error(e.getLocalizedMessage());
		            e.printStackTrace();
		        }
		        return timing;
		    }
		    
		
	//	@SuppressWarnings("null")
		public String call() {
//			int answer;
//			answer = getApplicationUI().displayModalDialog(
//			ApplicationDialogType.INFORMATION,"Moving Mode", "Manule","Handle");
			boolean DangerMove=false;
			int nLastWorkingmode=0;
			Frame Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
			Frame Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
			while (true)
			{ 
//				boolean btest=SafeDataIO.getInput4();
//				if (btest==true)
//				{
//					nWorkingmode=1;
//					System.out.println("SafeDataIO.getInput4"+btest);
//				}
//				System.out.println(btest);
				boolean btest=SafeDataIO.getInput4();
				if (btest==true)
				{
					nWorkingmode=1;
				}
				if (nWorkingmode==1){
					Err="0,";
					System.out.println("start handguiding");
					ThreadUtil.milliSleep(500);
					nLastWorkingmode=nWorkingmode;
					if(SafeDataIO.getInput4()==true)
					{
						bForPlane=true;
						ThreadUtil.milliSleep(500);
						DangerMove=false;
						if (nToolMode==2)
						{
							//nToolMode==2指的是安放模式
							needle_gripper.getFrame("/tcp").move(createhandGuidingMotion());
							bDangerous=false;
							nWorkingmode=0;
						}
						else{
							//其他指的是打磨模式
							needle.getFrame("/tcp_2").move(createhandGuidingMotion());
							bDangerous=false;
							nWorkingmode=0;
						}
						
		                nX=0;
		                nY=0;
		                nZ=0;
		                nA=0;
		                nB=0;
		                nC=0;
					}
					nWorkingmode=0;
					
				}
			    else if (nWorkingmode==2 ){
			    	nLastWorkingmode=nWorkingmode;
			    	DangerMove=false;
//					System.out.println("automode"+nWorkingmode);
//					Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();	
                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
					//$cmd,RobotMove,1$
			    	ThreadUtil.milliSleep(1000);
			    	final CartesianImpedanceControlMode cartImp = createCartImp();	
			    	if(nX==1)
			    	{
//			    		needle.getFrame("/tcp_2").move(ptp(Ptest1).setJointVelocityRel(0.2));
			    		System.out.println("zhunbei_ready");
			    		needle.getFrame("/tcp_2").move(new PTP(jointPos).setJointVelocityRel(0.2));	
			    		
						Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));

		                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
							//$cmd,RobotMove,1$
					
//								Ptest1.setX(-568.11);
//								Ptest1.setY(246.89);
//								Ptest1.setZ(481.69);
////								System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
//								Ptest1.setAlphaRad(Math.toRadians(151.47));
//								Ptest1.setBetaRad(Math.toRadians(-21.5));
//								Ptest1.setGammaRad(Math.toRadians(125.05));

								
							 //needle.getFrame("/tcp_2").move(ptp(Ptest1).setJointVelocityRel(0.2));	
			    	
			    	}
			    	//左侧
			    	else if(nX==2)
			    	{
			    		System.out.println("zuoce_ready");
			    		lbr.moveAsync(new PTP(jointPos).setJointVelocityRel(0.2).setMode(cartImp));
			    		lbr.moveAsync(new PTP(jointPos_zuo).setJointVelocityRel(0.2).setMode(cartImp));
			    	}
			    	//右侧
			    	else if(nX==3)
			    	{
			    		System.out.println("youce_ready");
			    		lbr.moveAsync(new PTP(jointPos).setJointVelocityRel(0.2).setMode(cartImp));
			    		lbr.moveAsync(new PTP(jointPos_you).setJointVelocityRel(0.2).setMode(cartImp));
			    	}
			    	else{
			    		System.out.println("err");
			    	}
//						Ptest1.setX(nX);
//						Ptest1.setY(nY);
//						Ptest1.setZ(nZ);
////						System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
//						Ptest1.setAlphaRad(Math.toRadians(nA));
//						Ptest1.setBetaRad(Math.toRadians(nB));
//						Ptest1.setGammaRad(Math.toRadians(nC));

			    	   
//					
//						
////						ThreadUtil.milliSleep(500);
////						System.out.println("222");
//						  System.out.println("*2");
//						JointPosition jReady =lbr.getCurrentJointPosition();
//						System.out.println("*3");
//		                jReady.set(1, -0.35);
//		                jReady.set(2, -0.71);
//		                jReady.set(3, 1.14);
//		                jReady.set(4, 0.92);
//		                jReady.set(5, -0.85);
//		                jReady.set(6, -1.607);
//		                jReady.set(7, 2.48);
//		                System.out.println(jReady);
//		                System.out.println("*4");
////						lbr.move(ptp(jReady).setMode(cartImp).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//		                lbr.move(ptp(jReady));
//		                System.out.println("*5");
		                
		                System.out.println("*2");
		                nWorkingmode=0;
		                System.out.println("*3");
		                nX=0;
		                nY=0;
		                nZ=0;
		                nA=0;
		                nB=0;
		                nC=0;
		        
		               
		        
//					Frame Ptest2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy().transform((Transformation.ofTranslation(0, 20, 0)));
			    	
			    	//随动模式
//					 getLogger().info("Move to start position.");
////				        lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
////				                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
//				        AbstractFrame initialPosition = lbr.getCurrentCartesianPosition(lbr
//				                .getFlange());
//				        // Create a new smart servo linear motion
//				        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);
//
//				        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);
//				        
//				        getLogger().info("Starting the SmartServoLIN in position control mode");
//				        lbr.getFlange().moveAsync(aSmartServoLINMotion);
//
//				        getLogger().info("Get the runtime of the SmartServoLIN motion");
//				        _smartServoLINRuntime = aSmartServoLINMotion.getRuntime();
//				        
//				        StatisticTimer timing = new StatisticTimer();
//
//				        // Start the smart servo lin sine movement
//				        timing = startSineMovement(_smartServoLINRuntime, timing);
//				  
//				        ThreadUtil.milliSleep(1000);
//
//				        getLogger().info("Print statistic timing");
//				        getLogger()
//				                .info(getClass().getName() + _smartServoLINRuntime.toString());
//
//				        getLogger().info("Stop the SmartServoLIN motion");
//				        _smartServoLINRuntime.stopMotion();
//
//				        // Statistic Timing of sine movement loop
//				        if (timing.getMeanTimeMillis() > 150)
//				        {
//				            getLogger()
//				                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
//				            getLogger()
//				                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
//				        }
				
				}
				
				else if(nWorkingmode==3){
					DangerMove=false;
					nLastWorkingmode=nWorkingmode;
//			        // Initialize Joint impedance mode    
//					System.out.println("JointimplentMode"+nWorkingmode);
//					moveToInitialPosition();
//			        final JointImpedanceControlMode jointImp = createJointImp();
//			        runSmartServoMotion(jointImp);
					
//					Frame Ptest1= getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();		
//					needle.getFrame("/tcp_2").move(ptp(Ptest1).setBlendingCart(0).setJointVelocityRel(0.2).setBlendingRel(0).setBlendingRel(0));
//					System.out.println("automode"+nWorkingmode);
//					Frame Ptest1= getApplicationData().getFrame("/P1").copyWithRedundancy();
					
					
					Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));

                   //testdata x:735  y:7.59  z:122 Aï¼š-91 Bï¼š-40 Cï¼š-178 $cmd,ml,715,7,122,-91,-40,-178$
					//$cmd,RobotMove,1$
					if(nX!=0 && nY!=0 && nZ!=0){
						Ptest1.setX(nX);
						Ptest1.setY(nY);
						Ptest1.setZ(nZ);
//						System.out.println("nx:"+nX+"  ny:"+nY+"  nz:"+nZ+"  a:"+nZ);
						Ptest1.setAlphaRad(Math.toRadians(nA));
						Ptest1.setBetaRad(Math.toRadians(nB));
						Ptest1.setGammaRad(Math.toRadians(nC));

						
						if(Math.abs(nX)<2000 && Math.abs(nY)<2000 && Math.abs(nZ)<2000 && Math.abs(nA)<2000 && Math.abs(nB)<2000 && Math.abs(nC)<2000){
							//System.out.println("pre_Place11***:"+pre_Place);
							needle.getFrame("/tcp_2").move(ptp(pre_Place).setJointVelocityRel(0.35));	
						}
						else{
							//System.out.println("Err_DangerPlace: "+"nX:"+nX+"nY:"+nY+"nZ:"+nZ+"nA:"+nA+"nB:"+nB+"nC:"+nC);
						}
						
						
//						ThreadUtil.milliSleep(500);
//						System.out.println("222");
					}
					else{
						ThreadUtil.milliSleep(10);
					}
					//nWorkingmode=0;

				}
				else if(nWorkingmode==4){
					nLastWorkingmode=nWorkingmode;
			        // Initialize Cartesian impedance mode    
//					System.out.println("CartesianimplentMode"+nWorkingmode);
//					moveToInitialPosition();
//					final CartesianImpedanceControlMode cartImp = createCartImp();
//			        runSmartCartesianMotion(cartImp);
               
			    	//圆锥打磨模式
			    	final CartesianImpedanceControlMode cartImp = ConeLimit();	
			    		
			    	System.out.println("ConeLimit");
			    	JointPosition jReady =lbr.getCurrentJointPosition();
//	                final JointPosition currentPos = lbr.getCurrentJointPosition();
	                
	     		   
	                if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 110 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
	                	if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160){
	                		System.out.println("J1:"+jReady.get(JointEnum.J1)+"   MAX:160;MIN:-160");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70){
	                		System.out.println("J2:"+jReady.get(JointEnum.J2)+"   MAX:70;MIN:-34");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55){
	                		System.out.println("J3:"+jReady.get(JointEnum.J3)+"   MAX:55;MIN:-65");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115){
	                		System.out.println("J4:"+jReady.get(JointEnum.J4)+"   MAX:110;MIN:-10");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J5)) < -160 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160){
	                		System.out.println("J5:"+jReady.get(JointEnum.J5)+"   MAX:160;MIN:-160");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J6)) > 110){
	                		System.out.println("J6:"+jReady.get(JointEnum.J6)+"   MAX:110;MIN:-110");
	                	}
	                	if(Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
	                		System.out.println("J7:"+jReady.get(JointEnum.J7)+"   MAX:165;MIN:-165");
	                	}
	                	System.out.println("dangermove1");
	                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
				    	needle.getFrame("/tcp_2").move(ptp(Ptest1).setJointVelocityRel(0.2));
				    	DangerMove=true;
	                }
	                else{
				    	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
				    	needle.getFrame("/tcp_2").move(ptp(Ptest1).setJointVelocityRel(0.2).setMode(cartImp));
	                }
	                

                    
			        // Initialize Joint impedance mode    
//					System.out.println("JointimplentMode"+nWorkingmode);
//					moveToInitialPosition();
//			        final JointImpedanceControlMode jointImp = createJointImp();
//			        runSmartServoMotion(jointImp);
	                
	                

			        nWorkingmode=0;

					
			        
			  if(bDangerous==true){
				  nWorkingmode=1;
				  System.out.println("WorkingModeForceToSet1");
			  }

			}
				//自动矫正点位
				else if(nWorkingmode==5){
		
					Frame Object=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));;
						
					ThreadUtil.milliSleep(1000);
//					Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					if (nToolMode==1){
						Object=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					else if(nToolMode==2){
						Object=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					
					int num=0;
					double nMinA=0,nMinB=0,nMinC=0;
					double nMinSun=10000000;
					double nObjectA=10000,nObjectB=100000,nObjectC=100000;
					double nObjectA_BackUp=10000,nObjectB_BackUp=100000,nObjectC_BackUp=100000;
//					Frame destObject1 = current.setX(current.getX());
//					System.out.println("current.getX():"+current.getX());
//					System.out.println("current.getA():"+current.getAlphaRad());
//					System.out.println("current.getB():"+current.getBetaRad());
					System.out.println("nA:"+nA+"  nB:"+nB+"  nC:"+nC);
					for (num = 0; num < 360; num = num + 1){
						//当前点位
						Frame current = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
						
						if (nToolMode==1){
//							current=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
							
							
//							cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//							System.out.println("1:"+cmdPos);
							Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
							cmdPos2.setX(0);
							cmdPos2.setY(0);
							cmdPos2.setZ(0);
							cmdPos2.setAlphaRad(0);
							cmdPos2.setBetaRad(Math.toRadians(-30));
							cmdPos2.setGammaRad(0);
							current=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"), cmdPos2);
//							System.out.println("current.getC():"+current);
							
						}
						else if(nToolMode==2){
//							current=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
							Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
							cmdPos2.setX(0);
							cmdPos2.setY(0);
							cmdPos2.setZ(0);
							cmdPos2.setAlphaRad(0);
							cmdPos2.setBetaRad(Math.toRadians(-30));
							cmdPos2.setGammaRad(0);
							current=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"), cmdPos2);
							System.out.println("current.getC():"+current);
						}
						
						Frame destObject = current.setX(current.getX());
						destObject = Object.setY(current.getY());
						destObject = Object.setZ(current.getZ());
						destObject = Object.setGammaRad(current.getGammaRad());		
						destObject = Object.setBetaRad(current.getBetaRad());
						destObject = Object.setAlphaRad(current.getAlphaRad());
						
						
//						Frame destObject = current.setX(-737.37);
//						destObject = Object.setY(216.62);
//						destObject = Object.setZ(232.57);
//						destObject = Object.setAlphaRad(-2.86530);
//						destObject = Object.setBetaRad(-0.096691);
//						destObject = Object.setGammaRad(2.3420570);		
						
					
						
						//System.out.println(Math.toDegrees(destObject.getBetaRad()));
						
						
						//test
						nA=-157.22;
						nB=22.9288;
						nC=157.359;
						
//						nA=164.7;
//						nB=-31.71;
//						nC=-164.418;
//						System.out.println("nA:"+nA+" nB:"+nB+" nC:"+nC);
						
//						System.out.println("pre_Place"+pre_Place);
//						System.out.println("nX"+nX+"  nY"+nY+"  nZ"+nZ+"  nA"+nA+"  nB"+nB+"  nC"+nC);
						
						
						//目标点位
						Frame destFrame = current.setX(nX);
						destFrame = current.setY(nY);
						destFrame = current.setZ(nZ);
						destFrame=  current.setAlphaRad(Math.toRadians(nA));
						destFrame=  current.setBetaRad(Math.toRadians(nB));
						destFrame=  current.setGammaRad(Math.toRadians(nC));
						
//						System.out.println("11"+destFrame);
//						
//						destFrame = current.setX(-714.741);
//						destFrame = current.setY(144.732);
//						destFrame = current.setZ(315.285);
//						destFrame = current.setAlphaRad(Math.toRadians(-139.612));
//						destFrame = current.setBetaRad(Math.toRadians(39.7936));
//						destFrame = current.setGammaRad(Math.toRadians(154.836));	
//						System.out.println("22"+destFrame);
						
						
						Frame Ptest1 = destFrame.transform((Transformation.ofDeg(0, 0, 0, num, 0, 0)));
						
						if(Math.toDegrees(Ptest1.getAlphaRad())>174 &&Math.toDegrees(Ptest1.getAlphaRad())<179){
							nObjectA_BackUp=Math.toDegrees(Ptest1.getAlphaRad());
							nObjectB_BackUp=Math.toDegrees(Ptest1.getBetaRad());
							nObjectC_BackUp=Math.toDegrees(Ptest1.getGammaRad());
						}
//						nMinA=Math.abs(Math.abs(Math.toDegrees(destObject.getAlphaRad()))-Math.abs(Math.toDegrees(Ptest1.getAlphaRad())));
//						nMinB=Math.abs(Math.abs(Math.toDegrees(destObject.getBetaRad()))-Math.abs(Math.toDegrees(Ptest1.getBetaRad())));
//						nMinC=Math.abs(Math.abs(Math.toDegrees(destObject.getGammaRad()))-Math.abs(Math.toDegrees(Ptest1.getGammaRad())));
						
						nMinA=Math.abs(Math.toDegrees(destObject.getAlphaRad())-Math.toDegrees(Ptest1.getAlphaRad()));
						nMinB=Math.abs(Math.toDegrees(destObject.getBetaRad())-Math.toDegrees(Ptest1.getBetaRad()));
						nMinC=Math.abs(Math.toDegrees(destObject.getGammaRad())-Math.toDegrees(Ptest1.getGammaRad()));
						
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
					System.out.println("a_BackUp:"+nObjectA_BackUp+" b_BackUp:"+nObjectB_BackUp+" c_BackUp:"+nObjectC_BackUp);
					Frame Object1=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					if (nToolMode==1){
						Object1=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					else if(nToolMode==2){
						Object1=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					
					//判断使用最优值还是备选值
					if(nObjectA>0){
						Object1.setAlphaRad(Math.toRadians(nObjectA));
						Object1.setBetaRad(Math.toRadians(nObjectB));
						Object1.setGammaRad(Math.toRadians(nObjectC));
					}
					else{
						Object1.setAlphaRad(Math.toRadians(nObjectA_BackUp));
						Object1.setBetaRad(Math.toRadians(nObjectB_BackUp));
						Object1.setGammaRad(Math.toRadians(nObjectC_BackUp));
					}

					System.out.println("x"+Object1.getX()+"y"+Object1.getY()+"z"+Object1.getZ()+"a:"+Object1.getAlphaRad()+" b:"+Object1.getBetaRad()+" c:"+Object1.getGammaRad());
					
					Frame Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					if (nToolMode==1){
						Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					else if(nToolMode==2){
						Object2=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					}
					System.out.println("111");
//					lbr.geti

					
//					Object2.setX(-737.34);
//					Object2.setY(216.63);
//					Object2.setZ(232.6);
//					Object2.setAlphaRad(Math.toRadians(-164.18));
//					Object2.setBetaRad(Math.toRadians(-5.54));
//					Object2.setGammaRad(Math.toRadians(134.19));
//					JointPosition test1=lbr.getInverseKinematicFromFrameAndRedundancy(Object2);
//					System.out.println(test1);
//					
//					Frame Object3 = Object2.transform((Transformation.ofDeg(0,0,-265.6, 0, 0, 0)));
//					Object3 = Object3.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
//					Object3 = Object3.transform((Transformation.ofDeg(-0.8 ,0,0, 0, 0, 0)));
//					System.out.println(Object3);
//					
//					Frame Object4 = Object2.transform((Transformation.ofDeg(-0.8,-168.5,-265.6, 0, 0, 59.999999931439994)));
////					Object4 = Object4.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
////					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,-265.6, 0, 0, 0)));
//					System.out.println(Object4);
//					
//			//		System.out.println("j1:"+test.get(1)+" j2"+test.get(2)+" j3"+test.get(3)+" j4"+test.get(4)+" j5"+test.get(5)+" j6"+test.get(6)+" j7"+test.get(7));
//				//	System.out.println("112");
					
//					Object2.setX(0);
//					Object2.setY(0);
//					Object2.setZ(0);
//					Object2.setAlphaRad(Math.toRadians(0));
//					Object2.setBetaRad(Math.toRadians(0));
//					Object2.setGammaRad(Math.toRadians(0));
//					JointPosition test1=lbr.getInverseKinematicFromFrameAndRedundancy(Object2);
//					System.out.println(test1);
//					System.out.println("Object21"+Object2);
////					Frame Object3 = Object2.transform((Transformation.ofDeg(0,0,-265.6, 0, 0, 0)));
////					Object3 = Object3.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
////					Object3 = Object3.transform((Transformation.ofDeg(-0.8 ,0,0, 0, 0, 0)));
////					System.out.println(Object3);
////					System.out.println("Object22"+Object2);
//					Frame Object4 = Object2.transform((Transformation.ofDeg(0.8,168.5,265.6, 0, 0, -59.999999931439994)));
////					Object4 = Object4.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
////					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,-265.6, 0, 0, 0)));
//					System.out.println("Object4"+Object4);
					
					
//					Object2.setX(-737.34);
//					Object2.setY(216.63);
//					Object2.setZ(232.6);
//					Object2.setAlphaRad(Math.toRadians(-164.18));
//					Object2.setBetaRad(Math.toRadians(-5.54));
//					Object2.setGammaRad(Math.toRadians(134.19));
//					JointPosition test1=lbr.getInverseKinematicFromFrameAndRedundancy(Object2);
//					System.out.println(test1);
//					System.out.println("Object21"+Object2);
////					Frame Object3 = Object2.transform((Transformation.ofDeg(0,0,-265.6, 0, 0, 0)));
////					Object3 = Object3.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
////					Object3 = Object3.transform((Transformation.ofDeg(-0.8 ,0,0, 0, 0, 0)));
////					System.out.println(Object3);
////					System.out.println("Object22"+Object2);
//					Frame Object4 = Object2.transform((Transformation.ofDeg(-0.8,-168.5,-265.6, 0, 0, 59.999999931439994)));
////					Object4 = Object4.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
////					Object4 = Object4.transform((Transformation.ofDeg(0 ,0,-265.6, 0, 0, 0)));
//					System.out.println("Object4"+Object4);
					

					JointPosition test1=lbr.getInverseKinematicFromFrameAndRedundancy(Object2);
					System.out.println(test1);
					System.out.println("Object21"+Object2);
//					Frame Object3 = Object2.transform((Transformation.ofDeg(0,0,-265.6, 0, 0, 0)));
//					Object3 = Object3.transform((Transformation.ofDeg(0,-168.5,0, 0, 0, 0)));
//					Object3 = Object3.transform((Transformation.ofDeg(-0.8 ,0,0, 0, 0, 0)));
//					System.out.println(Object3);
//					System.out.println("Object22"+Object2);
					//Frame Object4 = Object2.transform((Transformation.ofDeg(-0.8,-168.5,-265.6, 0, 0, 59.999999931439994)));
					//Frame Object4=lbr.getCurrentCartesianPosition(lbr.getFrame("/tcp_x_1_yz1"));
					Frame Object4 = Object2.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					//Object4 = Object4.transform((Transformation.ofDeg(-0.8,-168.5,-265.6, 0, 0, 0)));
					Object4 = Object4.transform((Transformation.ofDeg(0 ,-59.1,-141, 0, 0, 0)));
					System.out.println("Object4"+Object4);	
					try{
						
						lbr.getInverseKinematicFromFrameAndRedundancy(Object4);
						System.out.println("222");	
					Frame Object5 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
					System.out.println("333");	
					Object5.setX(Object4.getX());
					Object5.setY(Object4.getY());
					Object5.setZ(Object4.getZ());
					Object5.setAlphaRad(Object4.getAlphaRad());
					Object5.setBetaRad(Object4.getBetaRad());
					Object5.setGammaRad(Object4.getGammaRad());
					System.out.println("Object5"+Object5);
					JointPosition test=lbr.getInverseKinematicFromFrameAndRedundancy(Object5);
//					if(Math.toDegrees(test.get(JointEnum.J1))<15 && Math.toDegrees(test.get(JointEnum.J1))>-15 && Math.toDegrees(test.get(JointEnum.J2))>-15 && Math.toDegrees(test.get(JointEnum.J2) )<31 && Math.toDegrees(test.get(JointEnum.J3))>-46 && Math.toDegrees(test.get(JointEnum.J3))<46 && Math.toDegrees(test.get(JointEnum.J4))>-1 && Math.toDegrees(test.get(JointEnum.J4))<116 && Math.toDegrees(test.get(JointEnum.J5))>-66 && Math.toDegrees(test.get(JointEnum.J5))<66 &&Math.toDegrees(test.get(JointEnum.J6))>-111 && Math.toDegrees(test.get(JointEnum.J6))<-46 && Math.toDegrees(test.get(JointEnum.J7))>-166 && Math.toDegrees(test.get(JointEnum.J7))<166){
					if(true){
					
						if (nToolMode==1){
//							needle.getFrame("/tcp_2").move(lin(Object1).setJointVelocityRel(0.2));
							
							Frame pre_Point = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();

							
							
							Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
//							System.out.println("1:"+cmdPos);
							Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
							cmdPos2.setX(0);
							cmdPos2.setY(0);
							cmdPos2.setZ(0);
							cmdPos2.setAlphaRad(0);
							cmdPos2.setBetaRad(Math.toRadians(-30));
							cmdPos2.setGammaRad(0);
							cmdPos=lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"), cmdPos2);
							
							
							pre_Point.setX(cmdPos.getX());
							pre_Point.setY(cmdPos.getY());
							pre_Point.setZ(cmdPos.getZ());
							pre_Point.setAlphaRad(Object1.getAlphaRad());
							pre_Point.setBetaRad(Object1.getBetaRad());
							pre_Point.setGammaRad(Object1.getGammaRad());
							//用优化角度还是直接角度赋值	
//							pre_Point.setAlphaRad(Math.toRadians(nA));
//							pre_Point.setBetaRad(Math.toRadians(nB));
//							pre_Point.setGammaRad(Math.toRadians(nC));
							
							
							System.out.println("Object511111"+pre_Point);
							needle.getFrame("/tcp_x_1_yz1").move(lin(pre_Point).setJointVelocityRel(0.2));
							
						}
						else if(nToolMode==2){
							needle.getFrame("/tcp_x_1_yz1").move(lin(Object1).setJointVelocityRel(0.2));
						}
						
						System.out.println("InRange");
					}
					else{
						System.out.println("OutOfRange");
					}
					if(Math.toDegrees(test.get(JointEnum.J1))>11 || Math.toDegrees(test.get(JointEnum.J1))<-11){
						System.out.println("err1："+Math.toDegrees(test.get(JointEnum.J1)));
					}
					if(Math.toDegrees(test.get(JointEnum.J2))>31 || Math.toDegrees(test.get(JointEnum.J2))<-15){
						System.out.println("err2："+Math.toDegrees(test.get(JointEnum.J2)));
					}
					if(Math.toDegrees(test.get(JointEnum.J3))>46 || Math.toDegrees(test.get(JointEnum.J3))<-46){
						System.out.println("err3："+Math.toDegrees(test.get(JointEnum.J3)));
					}
					if(Math.toDegrees(test.get(JointEnum.J4))>116 || Math.toDegrees(test.get(JointEnum.J4))<-1){
						System.out.println("err4："+Math.toDegrees(test.get(JointEnum.J4)));
					}
					if(Math.toDegrees(test.get(JointEnum.J5))>66 || Math.toDegrees(test.get(JointEnum.J5))<-66){
						System.out.println("err5："+Math.toDegrees(test.get(JointEnum.J5)));
					}
					if(Math.toDegrees(test.get(JointEnum.J6))>-46 || Math.toDegrees(test.get(JointEnum.J6))<-111){
						System.out.println("err6："+Math.toDegrees(test.get(JointEnum.J6)));
					}
					if(Math.toDegrees(test.get(JointEnum.J7))>166 || Math.toDegrees(test.get(JointEnum.J7))<-166){
						System.out.println("err7："+Math.toDegrees(test.get(JointEnum.J7)));
					}
					System.out.println("J1："+Math.toDegrees(test.get(JointEnum.J1))+"   J2:"+Math.toDegrees(test.get(JointEnum.J2))+"   J3:"+Math.toDegrees(test.get(JointEnum.J3))+"   J4:"+Math.toDegrees(test.get(JointEnum.J4))+"   J5:"+Math.toDegrees(test.get(JointEnum.J5))+"   J6:"+Math.toDegrees(test.get(JointEnum.J6))+"   J7:"+Math.toDegrees(test.get(JointEnum.J7)) );
					}
					catch(Throwable cause)
					{
						System.out.println("OutOfRange2");
					}
					
					
					
			//		System.out.println("j1:"+test.get(1)+" j2"+test.get(2)+" j3"+test.get(3)+" j4"+test.get(4)+" j5"+test.get(5)+" j6"+test.get(6)+" j7"+test.get(7));
				//	System.out.println("112");
					
					
					
					
					nWorkingmode=0;
					
					
				}
				
				else if(nWorkingmode==6){
					nLastWorkingmode=nWorkingmode;
//			        // Initialize Joint impedance mode    
//					double nX1=-552;
//					double nY1=-56;
//					double nZ1=955;
//					double nA1=-71;
//					double nB1=44;
//					double nC1=54;
					ThreadUtil.milliSleep(3000);
//					if(nX!=0 && nY!=0 && nZ!=0){
                    if(true){   
//						if(num_ForTest!=0)
						if(true)
						{
							System.out.println("num_ForTest!=0");
							Frame pre_Place1 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();

							pre_Place1.setX(nX);
							pre_Place1.setY(nY);
							pre_Place1.setZ(nZ);
							pre_Place1.setAlphaRad(Math.toRadians(nA));
							pre_Place1.setBetaRad(Math.toRadians(nB));
							pre_Place1.setGammaRad(Math.toRadians(nC));
						
							Frame pre_Place2 = getApplicationData().getFrame("/CoverScrewing/SmallCover").copyWithRedundancy();
	 
							pre_Place2.setX(nX);
							pre_Place2.setY(nY);
							pre_Place2.setZ(nZ);
							pre_Place2.setAlphaRad(Math.toRadians(nA));
							pre_Place2.setBetaRad(Math.toRadians(nB));
							pre_Place2.setGammaRad(Math.toRadians(nC));
							
							
							
							
							System.out.println("pre_Place"+pre_Place1);
							Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
								if(nToolMode==3){
									Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
								}
								else if(nToolMode==2){
									Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz2"));
								}
			                	
						    	

					    		System.out.println("zhunbei_ready");
					    		Frame Object4 = Ptest1.transform((Transformation.ofDeg(0,0,0, 0, 0, 0)));
					    		Object4 = Object4.transform((Transformation.ofDeg(15 ,209,-124.5, 0, 0, 0)));
					    		System.out.println("Object4"+Object4);
					    		
					    		System.out.println(lbr.getInverseKinematicFromFrameAndRedundancy(Object4));
					    		lbr.getInverseKinematicFromFrameAndRedundancy(Object4);
								System.out.println("222");	
								Frame Object5 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
								
								if(nToolMode==3){
									Object5 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
								}
								else if(nToolMode==2){
									Object5 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz2"));
								}
								
								System.out.println("333");	
								Object5.setX(Object4.getX());
								Object5.setY(Object4.getY());
								Object5.setZ(Object4.getZ());
								Object5.setAlphaRad(Object4.getAlphaRad());
								Object5.setBetaRad(Object4.getBetaRad());
								Object5.setGammaRad(Object4.getGammaRad());
								System.out.println("Object5"+Object5);
								JointPosition test=lbr.getInverseKinematicFromFrameAndRedundancy(Object5);
								System.out.println("JointEnum.J1:"+test.get(JointEnum.J1));
					    		
					    		try{

									if(nToolMode==3){
										needle.getFrame("/tcp_x_1_yz4").move(lin(pre_Place2).setJointVelocityRel(0.1));
									}
									else if(nToolMode==2){
										needle.getFrame("/tcp_x_2_yz2").move(lin(pre_Place2).setJointVelocityRel(0.1));
									}
								
								//更新平面定位初始点
								
								if(nToolMode==3){
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
								}
								else if(nToolMode==2){
									Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
									Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
								}
//								num_ForTest=num_ForTest+5;
								Err="3,";
								System.out.println("Move");
								Err="3,";
								ThreadUtil.milliSleep(1000);
								Err="0,";
					    		}	
								catch(Throwable cause)
								{
									System.out.println("RangeLimit");
									Err="1,";
								}
							}
							else{
								 
							}
						}
					else{
						num_ForTest=1;
						System.out.println("num_ForTest==0");
					}
						

						
						

					
					nWorkingmode=0;
					
					
				}
				else if(nWorkingmode==99){
//					JointPosition currentPos_CheckSafety=lbr.getCurrentJointPosition();
//					if (Math.abs(Math.toDegrees(currentPos_CheckSafety.get(JointEnum.J7))) < 160){
//	            		System.out.println("ForPlane");
////						OnlyPlane.initialize();
//						OnlyPlane.run();
//						nWorkingmode=0;
//	                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//				    	needle.getFrame("/tcp_2").move(lin(Ptest1).setJointVelocityRel(0.2));
//	            	}
//					else{
//						Err="2,";
//					}
					
				}
				else if(nWorkingmode==7){
					
					
					
			    					
			    	final CartesianImpedanceControlMode carthard = HardLimit();
			    	final CartesianImpedanceControlMode carthard_Y = HardLimit_Y();
			    	final CartesianImpedanceControlMode carthard_Z = HardLimit_Z();
			    	if(nToolMode==3){
				    	Frame cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
				    	JointPosition test =lbr.getCurrentJointPosition();
			        	
		        		
		       	        Transformation DistanceToPlane=Ptest_ForPlane.staticTransformationTo(cmdPos2);
	        	   
		       	        count++;
		        	    nintegral=nintegral+DistanceToPlane.getX();
		         	    nderivative=DistanceToPlane.getX()-nPrevious_error;
		        	    nOutput=nP*DistanceToPlane.getX()+nI*nintegral+nD*nderivative;
		        	    nPrevious_error=DistanceToPlane.getX();
		            	Ptest_ForPlane1 = Ptest_ForPlane.copyWithRedundancy().transform((Transformation.ofTranslation(-nOutput, 0, 0))); 
		        	   
					    if(Math.abs(DistanceToPlane.getY())>130){
//					    	if(count%100==0){
					    		System.out.println("DistanceToPlane.getY())>100");
//					    	}
					    	
					    		cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
//					    	needle.getFrame("/tcp_x_1_yz3").moveAsync(ptp(cmdPos2).setJointVelocityRel(1).setMode(carthard_Y));
					    		Ptest_ForPlane1.setX(cmdPos2.getX());
					    		Ptest_ForPlane1.setY(cmdPos2.getY());
					    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
					    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
					    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
					    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
					    		needle.getFrame("/tcp_x_1_yz3").move(ptp(cmdPos2).setJointVelocityRel(0.1));
								Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
								Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    		nWorkingmode=0;
					    }
					    else if(Math.abs(DistanceToPlane.getZ())>130){
//					    	if(count%100==0){
					    		System.out.println("DgetZ()");
					    		Ptest_ForPlane1.setX(cmdPos2.getX());
					    		Ptest_ForPlane1.setY(cmdPos2.getY());
					    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
					    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
					    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
					    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
//					    	}
					
					        cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	needle.getFrame("/tcp_x_1_yz3").move(ptp(cmdPos2).setJointVelocityRel(0.1));
							Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
							Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	nWorkingmode=0;
					    }
					    else if(Math.toDegrees(test.get(JointEnum.J4)) > 105){
//					    	if(count%100==0){
					    		System.out.println("J4>115");
					    		Ptest_ForPlane1.setX(cmdPos2.getX());
					    		Ptest_ForPlane1.setY(cmdPos2.getY());
					    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
					    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
					    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
					    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
//					    	}
					
					        cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	needle.getFrame("/tcp_x_1_yz3").move(ptp(cmdPos2).setJointVelocityRel(0.1));
							Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
							Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	nWorkingmode=0;
					    }
				    else{
					    	needle.getFrame("/tcp_x_1_yz3").move(ptp(Ptest_ForPlane1).setJointVelocityRel(1).setMode(carthard));
					    	 
					    }
		            	
		            	if(count>100000){
		            		count=0;
		            	}
		            	if(count%30==0){
		            		 System.out.println("DistanceToPlane_x："+DistanceToPlane.getX()+"DistanceToPlane_c："+Math.toDegrees(DistanceToPlane.getGammaRad()));
		            	}
			    	}
			    	else if(nToolMode==2){
				    	Frame cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
		        		
			        	
		        		
		       	        Transformation DistanceToPlane=Ptest_ForPlane.staticTransformationTo(cmdPos2);
	        	   
		       	        count++;
		        	    nintegral=nintegral+DistanceToPlane.getX();
		         	    nderivative=DistanceToPlane.getX()-nPrevious_error;
		        	    nOutput=nP*DistanceToPlane.getX()+nI*nintegral+nD*nderivative;
		        	    nPrevious_error=DistanceToPlane.getX();
		            	Ptest_ForPlane1 = Ptest_ForPlane.copyWithRedundancy().transform((Transformation.ofTranslation(-nOutput, 0, 0))); 
		        	   
					    if(Math.abs(DistanceToPlane.getY())>120){
//					    	if(count%100==0){
					    		System.out.println("DistanceToPlane.getY())>100");
//					    	}
//					    	
					    		cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
//					    	needle.getFrame("/tcp_x_1_yz3").moveAsync(ptp(cmdPos2).setJointVelocityRel(1).setMode(carthard_Y));
					    		Ptest_ForPlane1.setX(cmdPos2.getX());
					    		Ptest_ForPlane1.setY(cmdPos2.getY());
					    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
					    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
					    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
					    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
					    		needle.getFrame("/tcp_x_1_yz3").move(ptp(cmdPos2).setJointVelocityRel(0.1));
								Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
								Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    		nWorkingmode=0;
					    }
					   
					    else if(Math.abs(DistanceToPlane.getZ())>120){
//					    	if(count%100==0){
					    		System.out.println("DgetZ()");
					    		Ptest_ForPlane1.setX(cmdPos2.getX());
					    		Ptest_ForPlane1.setY(cmdPos2.getY());
					    		Ptest_ForPlane1.setZ(cmdPos2.getZ());
					    		Ptest_ForPlane1.setGammaRad(cmdPos2.getGammaRad());
					    		Ptest_ForPlane1.setBetaRad(cmdPos2.getBetaRad());
					    		Ptest_ForPlane1.setAlphaRad(cmdPos2.getAlphaRad());
//					    	}
//					 
					        cmdPos2 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	needle.getFrame("/tcp_x_1_yz3").move(ptp(cmdPos2).setJointVelocityRel(0.1));
							Ptest_ForPlane = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
							Ptest_ForPlane1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
					    	nWorkingmode=0;
					    }
					    
				    else{
					    	needle.getFrame("/tcp_x_1_yz3").move(ptp(Ptest_ForPlane1).setJointVelocityRel(1).setMode(carthard));
//					    	ThreadUtil.milliSleep(200);
					    	 
					    }
		            	
		            	if(count>100000){
		            		count=0;
		            	}
		            	if(count%30==0){
		            		System.out.println("DistanceToPlane_x："+DistanceToPlane.getX()+"DistanceToPlane_c："+Math.toDegrees(DistanceToPlane.getGammaRad())); 
		                  //System.out.println("DistanceToPlane_x："+DistanceToPlane.getX()+"DistanceToPlane_Y："+DistanceToPlane.getY()+"DistanceToPlane_Z："+DistanceToPlane.getZ());
		            	}
			    	}
			    	else{
			    		ThreadUtil.milliSleep(2000);
			    		System.out.println("Err Tool");
			    	}
				    
//			    	ThreadUtil.milliSleep(200);
//			    	Ptest_ForPlane.setX(Ptest_ForPlane.getX()+1);
//			    	System.out.println("  nOutput:"+DistanceToPlane.getX());
			    	

			    

			    
			    
//			    	nWorkingmode=0;
				}
				else{
					
//					
					if(DangerMove==false && nLastWorkingmode==4){
						JointPosition jReady =lbr.getCurrentJointPosition();
//		                final JointPosition currentPos = lbr.getCurrentJointPosition();
		                
		     		   
//						 if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -25 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 10 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 110 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
//		                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//					    	needle.getFrame("/tcp_2").move(lin(Ptest1).setJointVelocityRel(0.2));
//					    	DangerMove=true;
//		                }
						 
			                if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J5)) < -160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160 ||  Math.toDegrees(jReady.get(JointEnum.J6)) > 110 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
			                	if(Math.toDegrees(jReady.get(JointEnum.J1)) < -160 || Math.toDegrees(jReady.get(JointEnum.J1)) > 160){
			                		System.out.println("J1:"+jReady.get(JointEnum.J1)+"   MAX:160;MIN:-160");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J2)) < -34 || Math.toDegrees(jReady.get(JointEnum.J2)) > 70){
			                		System.out.println("J2:"+jReady.get(JointEnum.J2)+"   MAX:70;MIN:-34");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J3)) < -65 || Math.toDegrees(jReady.get(JointEnum.J3)) > 55){
			                		System.out.println("J3:"+jReady.get(JointEnum.J3)+"   MAX:55;MIN:-65");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J4)) < -10 || Math.toDegrees(jReady.get(JointEnum.J4)) > 115){
			                		System.out.println("J4:"+jReady.get(JointEnum.J4)+"   MAX:110;MIN:-10");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J5)) < -160 || Math.toDegrees(jReady.get(JointEnum.J5)) > 160){
			                		System.out.println("J5:"+jReady.get(JointEnum.J5)+"   MAX:160;MIN:-160");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J6)) < -110 || Math.toDegrees(jReady.get(JointEnum.J6)) > 110){
			                		System.out.println("J6:"+jReady.get(JointEnum.J6)+"   MAX:110;MIN:-110");
			                	}
			                	if(Math.toDegrees(jReady.get(JointEnum.J7)) < -165 || Math.toDegrees(jReady.get(JointEnum.J7)) > 165){
			                		System.out.println("J7:"+jReady.get(JointEnum.J7)+"   MAX:165;MIN:-165");
			                	}
			                	Frame Ptest1 = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
						    	needle.getFrame("/tcp_2").move(lin(Ptest1).setJointVelocityRel(0.2));
						    	DangerMove=true;
			                	System.out.println("dangermove1");
			                }
			                	
						 
					}
					//ThreadUtil.milliSleep(4);
				}

			
			

			
			}
			//return "end get";
		}

	}

	
	
	
	
	//ä¿¡å�·é‡‡é›†
	public void Monitor() {
		
			while (true)
			{
			

				
				try{
				ThreadUtil.milliSleep(5000);

			
			
				
				//æš‚æ—¶æ— æ„�ä¹‰ï¼ˆé¢„ç•™é»˜è®¤ä¸º0ï¼‰
				data0 = "$0,";
				
				//æŠ¥è­¦ä»£ç �
				data1 = "0,";
				
				//å·¥ä½œæ¨¡å¼�
				data2 = "0,";
				
				//æ˜¯å�¦ä¸Šç”µ
				data3 = "1,";
				
				//å½“å‰�é€Ÿåº¦
				//CartesianVelocityLimitInfo infoObject = lbr.getCartesianVelocityLimitInfo();
				double a1 =getApplicationControl().getApplicationOverride();
				BigDecimal bigDecimal0 = new BigDecimal(a1);
				a1 = bigDecimal0.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data4 = String.valueOf(a1)+",";
				
				//æ— æ„�ä¹‰
				data5 = "0,";
				
//				//å…³èŠ‚åº¦æ•°1
				JointPosition actPos = lbr.getCurrentJointPosition();
				a1 = Math.toDegrees(actPos.get(0));
				BigDecimal bigDecimal = new BigDecimal(a1);
				a1 = bigDecimal.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data6=String.valueOf(a1)+",";
				
//				//å…³èŠ‚åº¦æ•°2
				a1 = Math.toDegrees(actPos.get(1));
				BigDecimal bigDecimal1 = new BigDecimal(a1);
				a1 = bigDecimal1.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data7=String.valueOf(a1)+",";
				
//				//å…³èŠ‚åº¦æ•°3
				a1 = Math.toDegrees(actPos.get(2));
				BigDecimal bigDecimal2 = new BigDecimal(a1);
				a1 = bigDecimal2.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data8=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°4
				a1 = Math.toDegrees(actPos.get(3));
				BigDecimal bigDecimal3 = new BigDecimal(a1);
				a1 = bigDecimal3.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data9=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°5
				a1 = Math.toDegrees(actPos.get(4));
				BigDecimal bigDecimal4 = new BigDecimal(a1);
				a1 = bigDecimal4.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data10=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°6
				a1 = Math.toDegrees(actPos.get(5));
				BigDecimal bigDecimal5 = new BigDecimal(a1);
				a1 = bigDecimal5.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data11=String.valueOf(a1)+",";
				
				//å…³èŠ‚åº¦æ•°7
				a1 = Math.toDegrees(actPos.get(6));
				BigDecimal bigDecimal6 = new BigDecimal(a1);
				a1 = bigDecimal6.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
				data12=String.valueOf(a1)+",";
				
				//è½´å��æ ‡x
				Frame cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
				cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
				System.out.println("1:"+cmdPos);
				Frame cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
				cmdPos2.setX(0);
				cmdPos2.setY(0);
				cmdPos2.setZ(0);
				cmdPos2.setAlphaRad(0);
				cmdPos2.setBetaRad(Math.toRadians(-30));
				cmdPos2.setGammaRad(0);
				cmdPos=lbr.getCommandedCartesianPosition(needle.getFrame("/tcp_2"), cmdPos2);
				System.out.println("2:"+cmdPos);
				
				Frame cmdPos1 = lbr.getCurrentCartesianPosition(needle.getFrame("/CoverScrewing"));
				System.out.println("getFlange()"+cmdPos1);
				
				
				
//				if (nToolMode==2){
//					 cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_anfang"));
//				}
//				else{
//					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
//				}
				
				if (nToolMode==1){
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));
					System.out.println("1:"+cmdPos);
					cmdPos2 = lbr.getCurrentCartesianPosition(lbr.getFlange());
					cmdPos2.setX(0);
					cmdPos2.setY(0);
					cmdPos2.setZ(0);
					cmdPos2.setAlphaRad(0);
					cmdPos2.setBetaRad(Math.toRadians(-30));
					cmdPos2.setGammaRad(0);
					cmdPos=lbr.getCommandedCartesianPosition(needle.getFrame("/tcp_2"), cmdPos2);
					System.out.println("2:"+cmdPos);
						
						
				}
				else if(nToolMode==2)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz1"));
				}	
				else if(nToolMode==3)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz2"));
				}	
				else if(nToolMode==4)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz3"));
				}	
				else if(nToolMode==5)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_1_yz4"));
				}	
				else if(nToolMode==6)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz2"));
				}	
				else if(nToolMode==7)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz3"));
				}	
				else if(nToolMode==8)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_2_yz4"));
				}	
				else if(nToolMode==9)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_3_yz3"));
				}	
				else if(nToolMode==10)
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_x_3_yz4"));
				}	
				else
				{
					cmdPos = lbr.getCurrentCartesianPosition(needle.getFrame("/test"));
				}
				
				a1=cmdPos.getX();
				BigDecimal bigDecimal7 = new BigDecimal(a1);
				a1 = bigDecimal7.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data13=a1+",";
				data13=String.valueOf(a1)+",";
				
				//è½´å��æ ‡y
				a1=cmdPos.getY();
				BigDecimal bigDecimal8 = new BigDecimal(a1);
				a1 = bigDecimal8.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data14=a1+",";
				data14=String.valueOf(a1)+",";
				
				//è½´å��æ ‡z
				a1=cmdPos.getZ();
				BigDecimal bigDecimal9 = new BigDecimal(a1);
				a1 = bigDecimal9.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data15=a1+",";
				data15=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rx
				a1=Math.toDegrees(cmdPos.getAlphaRad());
				BigDecimal bigDecima20 = new BigDecimal(a1);
				a1 = bigDecima20.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data16=a1+",";
				data16=String.valueOf(a1)+",";
				
				//è½´å��æ ‡ry
				a1=Math.toDegrees(cmdPos.getBetaRad());
				BigDecimal bigDecima21 = new BigDecimal(a1);
				a1 = bigDecima21.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data17=a1+",";
				data17=String.valueOf(a1)+",";
				
				//è½´å��æ ‡rz

				
				a1=Math.toDegrees(cmdPos.getGammaRad());
				BigDecimal bigDecima22 = new BigDecimal(a1);
				a1 = bigDecima22.setScale(2, BigDecimal.ROUND_HALF_UP).doubleValue();
//				data18=a1+"$";
				data18=String.valueOf(a1)+",$";
				}catch(Exception ex){
					break;
				}
			}

	}
	
	public boolean MotionType() {
		bForPlane=SafeDataIO.getInput4();
//		ThreadUtil.milliSleep(1000);
//		System.out.println("tttplane");
		return bForPlane;
	}

	
	//@SuppressWarnings("null")
	@Override
	public void run()  {

//		BreakTest.initialize();
//		BreakTest.run();
//		lbr.moveAsync(new PTP(jointPos_zuo).setJointVelocityRel(0.2));
//		ThreadUtil.milliSleep(2000);
		
//		OnlyPlane.run();
//		ThreadUtil.milliSleep(2000);
//		OnlyPlane.initialize();
//		OnlyPlane.run();
		
		
		ExecutorService executor = Executors.newCachedThreadPool();
		Future<String> add = executor.submit(new sendRTdata());
		Future<String> say = executor.submit(new motion());
		Future<String> sdd2 = executor.submit(new reciveRTdata());
        //Monitor();

		try {
			System.out.println(add.get());
			System.out.println(say.get());
			System.out.println(sdd2.get());
		} catch (InterruptedException e) {
			// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„ catch å�—
			e.printStackTrace();
		} catch (ExecutionException e) {
			// TODO è‡ªåŠ¨ç”Ÿæˆ�çš„ catch å�—
			e.printStackTrace();
		} 

		
		
	}
	@Override
	public void dispose(){
		try {
			ThreadUtil.milliSleep(1000);
			if(serverSocket!=null){
				serverSocket.close();
				serverSocket=null;
				System.out.println("222");
			}
			if(serverSocketSend!=null ){
				serverSocketSend.close();
				serverSocketSend=null;
				System.out.println("22222");
			}
			if(writer!=null ){
				writer.close();
				writer=null;
				System.out.println("3333");
			}
			if(outputStream!=null ){
				outputStream.close();
				outputStream=null;
				System.out.println("133333");
			}
			if(socket!=null ){
				socket.close();
				socket=null;
				System.out.println("233333");
			}
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
}