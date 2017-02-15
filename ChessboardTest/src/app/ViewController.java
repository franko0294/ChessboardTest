package app;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.WritableRaster;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import javafx.application.Platform;
import javafx.beans.property.ObjectProperty;
import javafx.embed.swing.SwingFXUtils;
import javafx.fxml.FXML;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

public class ViewController {
	@FXML
	private ImageView mainView;
	@FXML
	private ImageView mainViewCorrected;
	@FXML
	private ImageView secondView;
	@FXML
	private ImageView secondViewCorrected;
	
	private VideoCapture camera1;
	private VideoCapture camera2;
	
	private boolean camera1Started;
	private boolean camera2Started;
	
	private boolean camera1Found;
	private boolean camera2Found;
	
	private boolean getSecondCamera;
	private boolean registerCamera1;
	private boolean registerCamera2;
	
	private boolean camera1Calibrated;
	private boolean camera2Calibrated;
	
	private Mat camera1Frame;
	private Mat camera2Frame;
	
	private List<Mat> camera1Points;
	private List<Mat> camera2Points;
	private List<Mat> objectPoints;
	
	private MatOfPoint2f camera1Corners;
	private MatOfPoint2f camera2Corners;
	
	private MatOfPoint3f cameraObj;
	
	private Mat camera1Intrinsic;
	private Mat camera2Intrinsic;
	
	private Mat camera2Extrinsic;
	
	private Mat camera1Dist;
	private Mat camera2Dist;
	
	private ScheduledExecutorService timer;
	
	private Size boardSize;
	
	private int numFramesToCalib;
	private int numFrames1;
	private int numFrames2;
	
	private int numCornersHor;
	private int numCornersVer;
	private int numCorners;
	
	private long prevTime;
	private long currentTime;
	
	public ViewController() {
		
		camera1Started = false;
		camera1Found = false;
		camera2Started = false;
		camera2Found = false;
		
		getSecondCamera = false;
		registerCamera1 = false;
		registerCamera2 = false;
		
		//Init with size of calib chessboard
		numCornersHor = 9;
		numCornersVer = 14;
		numCorners = numCornersHor * numCornersVer;
		
		numFramesToCalib = 20;
		numFrames1 = 0;
		numFrames2 = 0;
		
		camera1Corners = new MatOfPoint2f();
		cameraObj = new MatOfPoint3f();
		
		camera2Corners = new MatOfPoint2f();
		
		camera1Intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		camera2Intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		
		camera2Extrinsic = new Mat(3, 3, CvType.CV_32FC1);
		
		camera1Dist = new Mat();
		camera2Dist = new Mat();
		
		camera1Calibrated = false;
		camera2Calibrated = false;
		
		prevTime = 0;
		currentTime = System.currentTimeMillis();
		
		for(int i = 0; i < numCorners; i++)
		{
			cameraObj.push_back(new MatOfPoint3f(new Point3(i / this.numCornersHor, i % this.numCornersVer, 0.0f)));
		}
	}
	
	@FXML
	private void startCamera1()
	{
		if(!camera1Started)
		{
			camera1 = new VideoCapture();
			camera1Started = camera1.open(0);
			
			if(camera1Started)
			{
				runCameras();
			}
			else
			{
				System.out.println("Error opening Camera 1, please check the connection and try again");
			}
		}
	}
	
	@FXML
	private void startCamera2()
	{
		if(camera1Started && !camera2Started)
		{
			camera2 = new VideoCapture();
			camera2Started = camera2.open(1);
			
			if(camera2Started)
			{
				getSecondCamera = true;
			}
			else
			{
				System.out.println("Error opening Camera 2, please check the connection and try again");
			}
		}
		else
		{
			System.out.println("Please start Camera 1 first");
		}
	}
	
	@FXML
	private void registerCamera1()
	{
		if(registerCamera1)
		{
			registerCamera1 = false;
		}
		else
		{
			registerCamera1 = true;
		}
	}
	
	@FXML
	private void registerCamera2()
	{
		if(registerCamera2)
		{
			registerCamera2 = false;
		}
		else
		{
			registerCamera2 = true;
		}
	}
	
	private void runCameras()
	{
		if(camera1Started)
		{
			Runnable framegrabber = new Runnable() 
			{
				
				@Override
				public void run() 
				{
					camera1Frame = new Mat();
					camera1.grab();
					camera1.retrieve(camera1Frame);
					
					//Imgproc.cvtColor(camera1Frame, camera1Frame, Imgproc.COLOR_BGR2GRAY);		
					
					if(getSecondCamera)
					{
						camera2Frame = new Mat();
						camera2.grab();
						camera2.retrieve(camera2Frame);
						
						//Imgproc.cvtColor(camera2Frame, camera2Frame, Imgproc.COLOR_BGR2GRAY);
					}
					
					if(registerCamera1)
					{
						findAndDrawPointsCam1(camera1Frame);
						
						if(camera1Calibrated)
						{
							
						}
					}
					
					updateImageView(mainView, mat2Image(camera1Frame));
					
					
					if(registerCamera2)
					{
						
					}
					else if(getSecondCamera)
					{
						updateImageView(secondView, mat2Image(camera2Frame));
					}
				}
			};
			
			timer = Executors.newSingleThreadScheduledExecutor();
			timer.scheduleAtFixedRate(framegrabber, 0, 100, TimeUnit.MILLISECONDS);
		}
	}
	
	private void findAndDrawPointsCam1(Mat frame)
	{
		Mat copy = new Mat();
		Imgproc.cvtColor(frame, copy, Imgproc.COLOR_BGR2GRAY);
		
		if(numFrames1 < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersHor, numCornersVer);
			
			boolean found = Calib3d.findChessboardCorners(copy, boardSize, camera1Corners, 
					Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
			
			if(found)
			{
				TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
				Imgproc.cornerSubPix(copy, camera1Corners, new Size(11, 11), new Size(-1, -1), term);
				// save the current frame for further elaborations
				// show the chessboard inner corners on screen
				Calib3d.drawChessboardCorners(frame, boardSize, camera1Corners, found);
				
				currentTime = System.currentTimeMillis();
				
				if(currentTime - prevTime > 500 && !camera1Calibrated)
				{
					takeSnapshotCam1();
					prevTime = currentTime;
				}
			}
		}
	}
	
	private void takeSnapshotCam1()
	{		
		if(numFrames1 < numFramesToCalib)
		{
			camera1Points.add(camera1Corners);
			objectPoints.add(cameraObj);
			numFrames1++;
		}
		
		if(numFrames1 == numFramesToCalib)
		{
			calibrateCam1();
		}
	}
	
	private void calibrateCam1()
	{
		List<Mat> rvecs = new ArrayList<>();
		List<Mat> tvecs = new ArrayList<>();
		camera1Intrinsic.put(0, 0, 1);
		camera1Intrinsic.put(1, 1, 1);
		
		double error = Calib3d.calibrateCamera(objectPoints, camera1Points, camera1Frame.size(), camera1Intrinsic, camera1Dist, rvecs, tvecs);
		
		System.out.println("Camera 1 error: " + error);
		
		camera1Calibrated = true;
	}
	
	private void findAndDrawPointsCam2(Mat frame)
	{
		Mat copy = frame.clone();
		
		if(numFrames2 < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersHor, numCornersVer);
			
			boolean found = Calib3d.findChessboardCorners(copy, boardSize, camera1Corners, 
					Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
			
			if(found)
			{
				TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
				Imgproc.cornerSubPix(copy, camera2Corners, new Size(11, 11), new Size(-1, -1), term);
				// save the current frame for further elaborations
				//copy.copyTo(this.savedImage);
				// show the chessboard inner corners on screen
				Calib3d.drawChessboardCorners(frame, boardSize, camera2Corners, found);
			}
		}
	}
	
	private void updateImageView(ImageView toUpdate, Image image)
	{
		onFXThread(toUpdate.imageProperty(), image);
	}

	private <T> void onFXThread(ObjectProperty<T> property, T value) {
		Platform.runLater(() -> {
			property.set(value);
		});
	}
	
	private Image mat2Image(Mat frame) {
		if(frame.type() != CvType.CV_8U)
		{
			frame.convertTo(frame, CvType.CV_8U);
		}
		try
		{
			return SwingFXUtils.toFXImage(matToBufferedImage(frame), null);
		}
		catch (Exception e)
		{
			System.err.println("Cannot convert the Mat obejct: " + e);
			return null;
		}
	}
	
	private BufferedImage matToBufferedImage(Mat original) {
		// init
		int type = 0;
		
		if(original.channels() == 1)
		{
			type = BufferedImage.TYPE_BYTE_GRAY;
		}
		else if(original.channels() == 3)
		{
			type = BufferedImage.TYPE_3BYTE_BGR;
		}
		
		BufferedImage image = new BufferedImage(original.width(), original.height(), type);
		WritableRaster raster = image.getRaster();
		DataBufferByte dataBuffer = (DataBufferByte) raster.getDataBuffer();
		
		byte[] data = dataBuffer.getData();
		
		original.get(0, 0, data);
		return image;
	}
}
