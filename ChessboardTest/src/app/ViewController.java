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
import org.opencv.core.Rect;
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
	
	private Mat camera1Undistorted;
	private Mat camera2Undistorted;
	
	private Mat camera1Map1;
	private Mat camera1Map2;
	private Mat camera2Map1;
	private Mat camera2Map2;
	
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
		numCornersHor = 6; //9
		numCornersVer = 9; //14
		numCorners = numCornersHor * numCornersVer;
		
		numFramesToCalib = 20;
		numFrames1 = 0;
		numFrames2 = 0;
		
		camera1Undistorted = new Mat();
		camera2Undistorted = new Mat();
		
		camera1Corners = new MatOfPoint2f();
		camera2Corners = new MatOfPoint2f();
		cameraObj = new MatOfPoint3f();
		
		camera1Points = new ArrayList<>();
		camera2Points = new ArrayList<>();
		objectPoints = new ArrayList<>();
		
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
			camera2 = new VideoCapture();
			camera2Started = camera2.open(1); 
			
			if(camera1Started && camera2Started)
			{
				getSecondCamera = true;
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
						findAndDrawPoints(camera1Frame, camera2Frame);
						
						if(camera1Calibrated && camera2Calibrated)
						{
							Imgproc.remap(camera1Frame, camera1Undistorted, camera1Map1, camera1Map2, Imgproc.INTER_NEAREST);
							
							//System.out.println("Showing camera 1");
							updateImageView(mainViewCorrected, mat2Image(camera1Undistorted));
							
							//System.out.println("Remapping camera 2");
							Imgproc.remap(camera2Frame, camera2Undistorted, camera2Map1, camera2Map2, Imgproc.INTER_LINEAR);
							
							//System.out.println(camera2Undistorted);
							
							//System.out.println("Showing camera 2");
							updateImageView(secondViewCorrected, mat2Image(camera2Undistorted));
						}
						
						/*
						if(camera1Calibrated)
						{
							if(camera2Calibrated)
							{
								System.out.println("Remapping camera 1");
								Imgproc.remap(camera1Frame, camera1Undistorted, camera1Map1, camera1Map2, Imgproc.INTER_NEAREST);
								
								System.out.println("Showing camera 1");
								updateImageView(mainViewCorrected, mat2Image(camera1Undistorted));
							}
							else
							{
								Imgproc.undistort(camera1Frame, camera1Undistorted, camera1Intrinsic, camera1Dist);
								
								//System.out.println(undistorted.dump());
								updateImageView(mainViewCorrected, mat2Image(camera1Undistorted));
							}
							
						}
						*/
					}
					
					updateImageView(mainView, mat2Image(camera1Frame));
					updateImageView(secondView, mat2Image(camera2Frame));
					
					/*
					if(registerCamera2)
					{
						findAndDrawPointsCam2(camera2Frame);
						
						if(camera2Calibrated)
						{
							System.out.println("Remapping camera 2");
							Imgproc.remap(camera2Frame, camera2Undistorted, camera2Map1, camera2Map2, Imgproc.INTER_LINEAR);
							
							System.out.println(camera2Undistorted);
							
							System.out.println("Showing camera 2");
							updateImageView(secondViewCorrected, mat2Image(camera2Undistorted));
							
							//Mat undistorted = new Mat();
							//Imgproc.undistort(camera2Frame, undistorted, camera2Intrinsic, camera2Dist);
							
							//System.out.println(undistorted.dump());
							//updateImageView(secondViewCorrected, mat2Image(undistorted));
						}
					}
					
					if(getSecondCamera)
					{
						updateImageView(secondView, mat2Image(camera2Frame));
					}
					*/
				}
			};
			
			timer = Executors.newSingleThreadScheduledExecutor();
			timer.scheduleAtFixedRate(framegrabber, 0, 100, TimeUnit.MILLISECONDS);
		}
	}
	
	private void findAndDrawPoints(Mat cam1, Mat cam2)
	{
		Mat copy1 = new Mat();
		Mat copy2 = new Mat();
		
		Imgproc.cvtColor(cam1, copy1, Imgproc.COLOR_BGR2GRAY);
		Imgproc.cvtColor(cam2, copy2, Imgproc.COLOR_BGR2GRAY);
		
		if(numFrames1 < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersVer, numCornersHor);
			
			boolean found1 = Calib3d.findChessboardCorners(copy1, boardSize, camera1Corners, 
					Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
			
			boolean found2 = Calib3d.findChessboardCorners(copy2, boardSize, camera2Corners, 
					Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
			
			if(found1 && found2)
			{
				TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
				Imgproc.cornerSubPix(copy1, camera1Corners, new Size(11, 11), new Size(-1, -1), term);
				Imgproc.cornerSubPix(copy2, camera2Corners, new Size(11, 11), new Size(-1, -1), term);
				// save the current frame for further elaborations
				// show the chessboard inner corners on screen
				Calib3d.drawChessboardCorners(cam1, boardSize, camera1Corners, found1);
				Calib3d.drawChessboardCorners(cam2, boardSize, camera2Corners, found2);
				
				currentTime = System.currentTimeMillis();
				
				if(currentTime - prevTime > 500 && !camera1Calibrated)
				{
					System.out.println("Taking snapshot " + numFrames1);
					takeSnapshot();
					prevTime = currentTime;
				}
			}
		}
	}
	
	private void findAndDrawPointsCam1(Mat frame)
	{
		Mat copy = new Mat();
		Imgproc.cvtColor(frame, copy, Imgproc.COLOR_BGR2GRAY);
		
		if(numFrames1 < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersVer, numCornersHor);
			
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
				
				if(currentTime - prevTime > 1000 && !camera1Calibrated)
				{
					System.out.println("Taking snapshot " + numFrames1);
					takeSnapshotCam1();
					prevTime = currentTime;
				}
			}
		}
	}
	
	private void takeSnapshot()
	{
		if(numFrames1 < numFramesToCalib)
		{
			camera1Points.add(camera1Corners);
			camera2Points.add(camera2Corners);
			objectPoints.add(cameraObj);
			
			numFrames1++;
		}
		
		if(numFrames1 == numFramesToCalib)
		{
			calibrateCameras();
		}
	}
	
	private void takeSnapshotCam1()
	{		
		if(numFrames1 < numFramesToCalib)
		{
			System.out.println("adding camera points");
			System.out.println(camera1Corners);
			camera1Points.add(camera1Corners);
			System.out.println("adding object points");
			objectPoints.add(cameraObj);
			numFrames1++;
		}
		
		System.out.println("Done taking snapshot");
		if(numFrames1 == numFramesToCalib)
		{
			calibrateCam1();
		}
	}
	
	private void takeSnapshotCam2()
	{	
		if(objectPoints.size() >= numFramesToCalib)
		{
			objectPoints.clear();
		}
		if(numFrames2 < numFramesToCalib)
		{
			System.out.println("adding camera points");
			System.out.println(camera2Corners);
			camera2Points.add(camera2Corners);
			System.out.println("adding object points");
			objectPoints.add(cameraObj);
			numFrames2++;
		}
		
		System.out.println("Done taking snapshot");
		if(numFrames2 == numFramesToCalib)
		{
			calibrateCam2();
		}
	}
	
	private void calibrateCameras()
	{
		List<Mat> rvecs1 = new ArrayList<>();
		List<Mat> rvecs2 = new ArrayList<>();
		
		List<Mat> tvecs1 = new ArrayList<>();
		List<Mat> tvecs2 = new ArrayList<>();
		
		camera1Intrinsic.put(0, 0, 1);
		camera1Intrinsic.put(1, 1, 1);
		
		camera2Intrinsic.put(0, 0, 1);
		camera2Intrinsic.put(1, 1, 1);
		
		double error1 = Calib3d.calibrateCamera(objectPoints, camera1Points, camera1Frame.size(), camera1Intrinsic, camera1Dist, rvecs1, tvecs1);
		
		double error2 = Calib3d.calibrateCamera(objectPoints, camera2Points, camera2Frame.size(), camera2Intrinsic, camera2Dist, rvecs2, tvecs2);
		
		System.out.println("Camera 1 error: " + error1);
		System.out.println("Camera 2 error: " + error2);
		
		Mat rotation = new Mat();
		Mat translation = new Mat();
		Mat essential = new Mat();
		Mat fundamental = new Mat();
		
		double stereoError = Calib3d.stereoCalibrate(objectPoints, camera1Points, camera2Points, camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist,
				camera2Frame.size(), rotation, translation, essential, fundamental, 
				Calib3d.CALIB_FIX_ASPECT_RATIO +
                Calib3d.CALIB_ZERO_TANGENT_DIST +
                Calib3d.CALIB_USE_INTRINSIC_GUESS +
                Calib3d.CALIB_SAME_FOCAL_LENGTH +
                Calib3d.CALIB_RATIONAL_MODEL +
                Calib3d.CALIB_FIX_K3 + Calib3d.CALIB_FIX_K4 + Calib3d.CALIB_FIX_K5,
                new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 100, 1e-5));
		
		System.out.println("Stereo error: " + stereoError);
		
		Mat rectify1 = new Mat();
		Mat rectify2 = new Mat();
		Mat projection1 = new Mat();
		Mat projection2 = new Mat();
		Mat Q = new Mat();
		
		Rect roi1 = new Rect();
		Rect roi2 = new Rect();
		
		//System.out.println("Rectifying");
		//Calib3d.stereoRectify(camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist, camera2Frame.size(), rotation, translation, 
		//		rectify1, rectify2, projection1, projection2, Q);
		Size newSize = new Size(camera1Frame.size().width, camera1Frame.size().height);
		Calib3d.stereoRectify(camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist, camera1Frame.size(), rotation, translation, 
				rectify1, rectify2, projection1, projection2, Q, 
				Calib3d.CALIB_ZERO_DISPARITY, 1, newSize,roi1, roi2);
		
		camera1Map1 = new Mat();
		camera1Map2 = new Mat();
		
		camera2Map1 = new Mat();
		camera2Map2 = new Mat();
		
		//System.out.println("initiating rectification");
		Imgproc.initUndistortRectifyMap(camera1Intrinsic, camera1Dist, rectify1, camera1Intrinsic, newSize, CvType.CV_16SC2, camera1Map1, camera1Map2);
		Imgproc.initUndistortRectifyMap(camera2Intrinsic, camera2Dist, rectify2, camera1Intrinsic, newSize, CvType.CV_16SC2, camera2Map1, camera2Map2);
		
		//System.out.println("Done with that shit");
		
		camera1Calibrated = true;
		camera2Calibrated = true;
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
	
	private void calibrateCam2()
	{
		List<Mat> rvecs = new ArrayList<>();
		List<Mat> tvecs = new ArrayList<>();
		camera2Intrinsic.put(0, 0, 1);
		camera2Intrinsic.put(1, 1, 1);
		
		double error = Calib3d.calibrateCamera(objectPoints, camera2Points, camera2Frame.size(), camera2Intrinsic, camera2Dist, rvecs, tvecs);
		
		System.out.println("Camera 2 error: " + error);
		
		Mat rotation = new Mat();
		Mat translation = new Mat();
		Mat essential = new Mat();
		Mat fundamental = new Mat();
		
		double stereoError = Calib3d.stereoCalibrate(objectPoints, camera1Points, camera2Points, camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist,
				camera2Frame.size(), rotation, translation, essential, fundamental);
		
		System.out.println("Stereo error: " + stereoError);
		
		Mat rectify1 = new Mat();
		Mat rectify2 = new Mat();
		Mat projection1 = new Mat();
		Mat projection2 = new Mat();
		Mat Q = new Mat();
		
		Rect roi1 = new Rect();
		Rect roi2 = new Rect();
		
		System.out.println("Rectifying");
		//Calib3d.stereoRectify(camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist, camera2Frame.size(), rotation, translation, 
		//		rectify1, rectify2, projection1, projection2, Q);
		Calib3d.stereoRectify(camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist, camera1Frame.size(), rotation, translation, 
				rectify1, rectify2, projection1, projection2, Q, 
				Calib3d.CALIB_ZERO_DISPARITY, 1, camera1Frame.size(), roi1, roi2);
		
		camera1Map1 = new Mat();
		camera1Map2 = new Mat();
		
		camera2Map1 = new Mat();
		camera2Map2 = new Mat();
		
		System.out.println("initiating rectification");
		Imgproc.initUndistortRectifyMap(camera1Intrinsic, camera1Dist, rectify1, camera1Intrinsic, camera1Frame.size(), CvType.CV_16SC2, camera1Map1, camera1Map2);
		Imgproc.initUndistortRectifyMap(camera2Intrinsic, camera2Dist, rectify2, camera1Intrinsic, camera2Frame.size(), CvType.CV_16SC2, camera2Map1, camera2Map2);
		
		System.out.println("Done with that shit");
		
		camera2Calibrated = true;
	}
	
	private void findAndDrawPointsCam2(Mat frame)
	{
		Mat copy = new Mat();
		Imgproc.cvtColor(frame, copy, Imgproc.COLOR_BGR2GRAY);
		
		if(numFrames2 < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersVer, numCornersHor);
			
			boolean found = Calib3d.findChessboardCorners(copy, boardSize, camera2Corners, 
					Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);
			
			if(found)
			{
				TermCriteria term = new TermCriteria(TermCriteria.EPS | TermCriteria.MAX_ITER, 30, 0.1);
				Imgproc.cornerSubPix(copy, camera2Corners, new Size(11, 11), new Size(-1, -1), term);
				// save the current frame for further elaborations
				// show the chessboard inner corners on screen
				Calib3d.drawChessboardCorners(frame, boardSize, camera2Corners, found);
				
				currentTime = System.currentTimeMillis();
				
				if(currentTime - prevTime > 1000 && !camera2Calibrated)
				{
					System.out.println("Taking snapshot");
					takeSnapshotCam2();
					prevTime = currentTime;
				}
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
