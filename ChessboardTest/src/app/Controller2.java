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
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import javafx.application.Platform;
import javafx.beans.property.ObjectProperty;
import javafx.embed.swing.SwingFXUtils;
import javafx.fxml.FXML;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;

public class Controller2 {
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
	
	private Mat frame1;
	private Mat frame2;
	
	private boolean camera1Calibrated;
	private boolean camera2Calibrated;
	
	private List<Mat> camera1Points;
	private List<Mat> camera2Points;
	private List<Mat> objectPoints;
	
	private MatOfPoint2f camera1Corners;
	private MatOfPoint2f camera2Corners;
	
	private MatOfPoint3f cameraObj;
	
	private Mat camera1Intrinsic;
	private Mat camera2Intrinsic;
	
	private Mat camera1Dist;
	private Mat camera2Dist;
	
	private int numFramesToCalib;
	private int numFrames;
	
	private int numCornersHor;
	private int numCornersVer;
	private int numCorners;
	
	private long prevTime;
	private long currentTime;
	
	private ScheduledExecutorService timer;
	
	private boolean registerCameras;
	
	public Controller2() {
		camera1 = new VideoCapture(0);
		camera2 = new VideoCapture(1);
		
		registerCameras = false;
		
		numCornersHor = 4;
		numCornersVer = 11;
		numCorners = numCornersHor * numCornersVer;
		
		camera1Corners = new MatOfPoint2f();
		camera2Corners = new MatOfPoint2f();
		cameraObj = new MatOfPoint3f();
		
		camera1Intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		camera2Intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		
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
	private void startCameras()
	{
		if(!camera1.isOpened() && !camera2.isOpened())
		{
			if(camera1.open(0) && camera2.open(1))
			{
				runCameras();
			}
			else
			{
				System.out.println("Cannot open cameras, please try again");
			}
		}
	}
	
	@FXML
	private void registerCameras()
	{
		if(registerCameras)
		{
			registerCameras = false;
		}
		else
		{
			registerCameras = true;
		}
	}
	
	private void runCameras()
	{
		if(camera1.isOpened() && camera2.isOpened())
		{
			Runnable framegrabber = new Runnable() {
				
				@Override
				public void run() {
					frame1 = new Mat();
					frame2 = new Mat();
					
					camera1.read(frame1);
					camera2.read(frame2);
					
					if(registerCameras)
					{
						//Calibrate cameras individually using assymetrical circles grid
						findAndDrawPoints(frame1, frame2);
						//Get perspective transform based off the same 4 points on the circle grid
						if(camera1Calibrated && camera2Calibrated)
						{
							Mat frame1UD = new Mat();
							Mat frame2UD = new Mat();
							Calib3d.undistortImage(frame1, frame1UD, camera1Intrinsic, camera1Dist);	
							Calib3d.undistortImage(frame2, frame2UD, camera2Intrinsic, camera2Dist);
							
							updateImageView(mainViewCorrected, mat2Image(frame1UD));
							updateImageView(secondViewCorrected, mat2Image(frame2UD));
						}
					}
					
					updateImageView(mainView, mat2Image(frame1));
					updateImageView(secondView, mat2Image(frame2));
				}
			};
			
			timer = Executors.newSingleThreadScheduledExecutor();
			timer.scheduleAtFixedRate(framegrabber, 0, 100, TimeUnit.MILLISECONDS);
		}
	}
	
	private void findAndDrawPoints(Mat frame1, Mat frame2)
	{
		Mat copy1 = new Mat();
		Imgproc.cvtColor(frame1, copy1, Imgproc.COLOR_BGR2GRAY);
		
		Mat copy2 = new Mat();
		Imgproc.cvtColor(frame2, copy2, Imgproc.COLOR_BGR2GRAY);
		
		if(numFrames < numFramesToCalib)
		{
			Size boardSize = new Size(numCornersHor, numCornersVer);
			
			boolean found1 = Calib3d.findCirclesGrid(copy1, boardSize, camera1Corners, Calib3d.CALIB_CB_ASYMMETRIC_GRID);
			boolean found2 = Calib3d.findCirclesGrid(copy2, boardSize, camera1Corners, Calib3d.CALIB_CB_ASYMMETRIC_GRID);
			
			if(found1 && found2)
			{
				Calib3d.drawChessboardCorners(frame1, boardSize, camera1Corners, found1);
				Calib3d.drawChessboardCorners(frame2, boardSize, camera2Corners, found2);
				
				currentTime = System.currentTimeMillis();
				
				if(currentTime - prevTime > 500 && !camera1Calibrated)
				{
					takeSnapshot();
					prevTime = currentTime;
				}
			}
		}
	}
	
	private void takeSnapshot()
	{
		if(numFrames < numFramesToCalib)
		{
			camera1Points.add(camera1Corners);
			camera2Points.add(camera2Corners);
			
			objectPoints.add(cameraObj);
			numFrames++;
		}
		
		if(numFrames == numFramesToCalib)
		{
			calibrateCameras();
		}
	}
	
	private void calibrateCameras()
	{
		List<Mat> rvecs1 = new ArrayList<>();
		List<Mat> tvecs1 = new ArrayList<>();
		List<Mat> rvecs2 = new ArrayList<>();
		List<Mat> tvecs2 = new ArrayList<>();
		
		camera1Intrinsic.put(0, 0, 1);
		camera1Intrinsic.put(1, 1, 1);
		camera2Intrinsic.put(0, 0, 1);
		camera2Intrinsic.put(1, 1, 1);
		
		double error1 = Calib3d.calibrateCamera(objectPoints, camera1Points, frame1.size(), camera1Intrinsic, camera1Dist, rvecs1, tvecs1);
		double error2 = Calib3d.calibrateCamera(objectPoints, camera2Points, frame2.size(), camera2Intrinsic, camera2Dist, rvecs2, tvecs2);
		
		System.out.println("Camera 1 error: " + error1 + "\nCamera 2 error: " + error2);
		
		camera1Calibrated = true;
		camera2Calibrated = true;
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

