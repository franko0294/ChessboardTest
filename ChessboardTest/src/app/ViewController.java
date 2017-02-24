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
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.opencv.videoio.VideoCapture;
import org.opencv.videoio.Videoio;

import javafx.application.Platform;
import javafx.beans.property.ObjectProperty;
import javafx.embed.swing.SwingFXUtils;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
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
	@FXML
	private Button snapButton;
	
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
	private List<MatOfPoint2f> camera1Points2f;
	private List<Mat> camera2Points;
	private List<MatOfPoint2f> camera2Points2f;
	private List<Mat> objectPoints;
	private List<MatOfPoint3f> objectPoints3f;
	
	private MatOfPoint2f camera1Corners;
	private MatOfPoint2f camera2Corners;
	
	private MatOfPoint3f cameraObj;
	
	private Mat camera1Intrinsic;
	private Mat camera2Intrinsic;
	
	private Mat camera1Dist;
	private Mat camera2Dist;
	
	private ScheduledExecutorService timer;
	
	private int numFramesToCalib;
	private int numFrames1;
	
	private int numCornersHor;
	private int numCornersVer;
	
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
		
		numFramesToCalib = 20;
		numFrames1 = 0;
		
		camera1Undistorted = new Mat();
		camera2Undistorted = new Mat();
		
		camera1Corners = new MatOfPoint2f();
		camera2Corners = new MatOfPoint2f();
		cameraObj = new MatOfPoint3f();
		
		camera1Points = new ArrayList<>();
		camera1Points2f = new ArrayList<>();
		camera2Points = new ArrayList<>();
		camera2Points2f = new ArrayList<>();
		objectPoints = new ArrayList<>();
		objectPoints3f = new ArrayList<>();
		
		camera1Intrinsic = new Mat(3, 3, CvType.CV_32FC1);
		camera2Intrinsic = new Mat(3, 3, CvType.CV_32FC1);

		camera1Dist = new Mat();
		camera2Dist = new Mat();
		
		camera1Calibrated = false;
		camera2Calibrated = false;
		
		prevTime = 0;
		currentTime = System.currentTimeMillis();
		
		for(int i = 0; i < numCornersHor; i++)
		{
			for(int j = 0; j < numCornersVer; j++)
			{
				cameraObj.push_back(new MatOfPoint3f(new Point3(i, j, 0)));
			}
		}
		/*
		for(int i = 0; i < numCorners; i++)
		{
			cameraObj.push_back(new MatOfPoint3f(new Point3(i / this.numCornersHor, i % this.numCornersVer, 0.0f)));
		}
		*/
	}
	
	@FXML
	private void startCameras()
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
	private void registerCameras()
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
							
//							System.out.println("Remapping camera 1");
							Imgproc.remap(camera1Frame, camera1Undistorted, camera1Map1, camera1Map2, Imgproc.INTER_LINEAR);
							
//							System.out.println("Showing camera 1");
							updateImageView(mainViewCorrected, mat2Image(camera1Undistorted));
							
//							System.out.println("Remapping camera 2");
							Imgproc.remap(camera2Frame, camera2Undistorted, camera2Map1, camera2Map2, Imgproc.INTER_LINEAR);
							
							//System.out.println(camera2Undistorted);
							
//							System.out.println("Showing camera 2");
							updateImageView(secondViewCorrected, mat2Image(camera2Undistorted));
						
						}
					}
					
					updateImageView(mainView, mat2Image(camera1Frame));
					updateImageView(secondView, mat2Image(camera2Frame));
				}
			};
			
			timer = Executors.newSingleThreadScheduledExecutor();
			timer.scheduleAtFixedRate(framegrabber, 0, 33, TimeUnit.MILLISECONDS);
		}
	}
	
	private void findAndDrawPoints(Mat cam1, Mat cam2)
	{
		Mat copy1 = new Mat();
		Mat copy2 = new Mat();
		
		Imgproc.cvtColor(cam1, copy1, Imgproc.COLOR_BGR2GRAY);
		Imgproc.cvtColor(cam2, copy2, Imgproc.COLOR_BGR2GRAY);
		
		Size boardSize = new Size(numCornersVer, numCornersHor);
		
		boolean found1 = Calib3d.findChessboardCorners(copy1, boardSize, camera1Corners, 
				Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE);
		
		boolean found2 = Calib3d.findChessboardCorners(copy2, boardSize, camera2Corners, 
				Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE);
		
		if(found1 && found2)
		{
			TermCriteria term = new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, 0.01);
			Imgproc.cornerSubPix(copy1, camera1Corners, new Size(11, 11), new Size(-1, -1), term);
			Imgproc.cornerSubPix(copy2, camera2Corners, new Size(11, 11), new Size(-1, -1), term);
			// save the current frame for further elaborations
			// show the chessboard inner corners on screen
			Calib3d.drawChessboardCorners(cam1, boardSize, camera1Corners, found1);
			Calib3d.drawChessboardCorners(cam2, boardSize, camera2Corners, found2);	
			
		}
	}
	
	@FXML
	private void takeSnapshot()
	{
		if(numFrames1 < numFramesToCalib)
		{
			camera1Points.add(camera1Corners);
			camera1Points2f.add(camera1Corners);
			camera2Points.add(camera2Corners);
			camera2Points2f.add(camera2Corners);
			objectPoints.add(cameraObj);
			objectPoints3f.add(cameraObj);
			
			numFrames1++;
		}
		
		if(numFrames1 == numFramesToCalib)
		{
			//stereoUncalibrated();
			calibrateCameras();
		}
	}
	
	private void stereoUncalibrated()
	{
		System.out.println("Getting fundamental mat");
		
		ArrayList<Mat> temp = cloneArrayList(camera1Points);
		
		Mat temp2f = Converters.vector_Mat_to_Mat(temp);
		
		temp = cloneArrayList(objectPoints);
		Mat temp3f = Converters.vector_Mat_to_Mat(temp);
		//System.out.println("test");
		List<MatOfPoint2f> temparray2f = new ArrayList<>();
		List<MatOfPoint3f> temparray3f = new ArrayList<>();
		
		Converters.Mat_to_vector_vector_Point2f(temp2f, temparray2f);
		Converters.Mat_to_vector_vector_Point3f(temp3f, temparray3f);
		
		//System.out.println("Temp points\n" + temparray2f.get(0).dump());
		
		System.out.println("Object points:\n" + objectPoints.get(0));
		
		camera1Intrinsic = Calib3d.initCameraMatrix2D(temparray3f, temparray2f, camera1Frame.size());
		
		temp = cloneArrayList(camera2Points);
		
		temp2f = Converters.vector_Mat_to_Mat(temp);
		temparray2f.clear();
		Converters.Mat_to_vector_vector_Point2f(temp2f, temparray2f);
		
		camera2Intrinsic = Calib3d.initCameraMatrix2D(temparray3f, temparray2f, camera2Frame.size());
		
		MatOfPoint2f totalCorners1 = new MatOfPoint2f();
		
		for (Mat mat : camera1Points) {
			totalCorners1.push_back(mat);
		}
		
		MatOfPoint2f totalCorners2 = new MatOfPoint2f();
		
		for (Mat mat : camera2Points) {
			totalCorners2.push_back(mat);
		}
		
		Mat F = Calib3d.findFundamentalMat(totalCorners1, totalCorners2, Calib3d.FM_8POINT, 0, 0);
		Mat H1 = new Mat(4, 4, camera1Frame.type());
		Mat H2 = new Mat(4, 4, camera2Frame.type());
		
		System.out.println(F.dump());
		
		Size newSize = new Size(camera1Frame.size().width, camera1Frame.size().height);
		
		//System.out.println("Rectifying");
		boolean done = Calib3d.stereoRectifyUncalibrated(camera1Corners, camera2Corners, F, newSize,H1, H2);
		
		camera1Map1 = new Mat();
		camera1Map2 = new Mat();
		
		camera2Map1 = new Mat();
		camera2Map2 = new Mat();
		
		Mat rectify1 = new Mat();
		
		Core.multiply(camera1Intrinsic.inv(), H1, rectify1);
		Core.multiply(rectify1, camera1Intrinsic, rectify1);
		
		Mat rectify2 = new Mat();
		
		Core.multiply(camera2Intrinsic.inv(), H2, rectify2);
		Core.multiply(rectify1, camera2Intrinsic, rectify2);
		
		//System.out.println("initiating rectification");
		Imgproc.initUndistortRectifyMap(camera1Intrinsic, camera1Dist, rectify1, camera1Intrinsic, newSize, CvType.CV_16SC2, camera1Map1, camera1Map2);
		Imgproc.initUndistortRectifyMap(camera2Intrinsic, camera2Dist, rectify2, camera1Intrinsic, newSize, CvType.CV_16SC2, camera2Map1, camera2Map2);
		
		snapButton.setDisable(true);
		
		camera1Calibrated = true;
		camera2Calibrated = true;
		//System.out.println(camera1Map1.dump());
		
		//System.out.println("Done rectifying: " + done);
		//Imgproc.warpPerspective(camera1Frame, camera1Frame, H1, camera1Frame.size());
	}
	
	private void calibrateCameras()
	{
		camera1Intrinsic = Calib3d.initCameraMatrix2D(objectPoints3f, camera1Points2f, camera1Frame.size());
		camera2Intrinsic = Calib3d.initCameraMatrix2D(objectPoints3f, camera2Points2f, camera2Frame.size());
		
		Mat rotation = new Mat();
		Mat translation = new Mat();
		Mat essential = new Mat();
		Mat fundamental = new Mat();
		
		System.out.println("Test");
		double stereoError = Calib3d.stereoCalibrate(objectPoints, camera1Points, camera2Points, camera1Intrinsic, camera1Dist, camera2Intrinsic, camera2Dist,
				camera1Frame.size(), rotation, translation, essential, fundamental, 
				Calib3d.CALIB_FIX_ASPECT_RATIO +
                Calib3d.CALIB_ZERO_TANGENT_DIST +
                Calib3d.CALIB_USE_INTRINSIC_GUESS +
                Calib3d.CALIB_SAME_FOCAL_LENGTH +
                Calib3d.CALIB_RATIONAL_MODEL +
                Calib3d.CALIB_FIX_K3 + 
                Calib3d.CALIB_FIX_K4 + 
                Calib3d.CALIB_FIX_K5,
                new TermCriteria(TermCriteria.COUNT + TermCriteria.EPS, 30, 0.01));
		
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
				Calib3d.CALIB_ZERO_DISPARITY, 0, newSize,roi1, roi2);
		
		camera1Map1 = new Mat();
		camera1Map2 = new Mat();
		
		camera2Map1 = new Mat();
		camera2Map2 = new Mat();
		
		//System.out.println("initiating rectification");
		Imgproc.initUndistortRectifyMap(camera1Intrinsic, camera1Dist, rectify1, projection1, newSize, CvType.CV_16SC2, camera1Map1, camera1Map2);
		Imgproc.initUndistortRectifyMap(camera2Intrinsic, camera2Dist, rectify2, projection2, newSize, CvType.CV_16SC2, camera2Map1, camera2Map2);
		
		//System.out.println("Done with that shit");
		
		camera1Calibrated = true;
		camera2Calibrated = true;
	}
	
	private ArrayList<Mat> cloneArrayList(List<Mat> camera1Points2)
	{
		ArrayList<Mat> clone = new ArrayList<>();
		
		for (Mat item : camera1Points2) {
			clone.add(item.clone());
		}
		
		return clone;
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
