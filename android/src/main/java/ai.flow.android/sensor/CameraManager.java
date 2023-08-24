package ai.flow.android.sensor;

import ai.flow.common.ParamsInterface;
import ai.flow.common.transformations.Camera;
import ai.flow.common.utils;
import ai.flow.definitions.Definitions;
import ai.flow.modeld.ModelExecutorF3;
import ai.flow.modeld.messages.MsgFrameData;
import ai.flow.sensor.SensorInterface;
import ai.flow.sensor.messages.MsgFrameBuffer;

import android.annotation.SuppressLint;
import android.content.Context;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.MeteringRectangle;
import android.hardware.camera2.params.TonemapCurve;
import android.os.Build;
import android.util.Range;
import android.util.Size;

import androidx.annotation.NonNull;
import androidx.annotation.OptIn;
import androidx.annotation.RequiresApi;
import androidx.camera.camera2.interop.Camera2Interop;
import androidx.camera.core.*;
import androidx.camera.lifecycle.ProcessCameraProvider;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import com.google.common.util.concurrent.ListenableFuture;

import messaging.ZMQPubHandler;
import org.capnproto.PrimitiveList;
import org.opencv.core.Core;

import java.io.File;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.List;
import java.util.TimeZone;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static ai.flow.android.sensor.Utils.fillYUVBuffer;
import static ai.flow.common.BufferUtils.byteToFloat;
import static ai.flow.common.transformations.Camera.CAMERA_TYPE_ROAD;
import static ai.flow.common.transformations.Camera.CAMERA_TYPE_WIDE;

public class CameraManager extends SensorInterface {

    private ImageAnalysis.Analyzer myAnalyzer, roadAnalyzer = null;
    //public static List<CameraManager> Managers = new ArrayList<>();
    public ProcessCameraProvider cameraProvider;
    public String frameDataTopic, frameBufferTopic, intName;
    public ZMQPubHandler ph;
    ExecutorService threadpool;
    public boolean running = false;
    public int W = Camera.frameSize[0];
    public int H = Camera.frameSize[1];
    public MsgFrameData msgFrameData, msgFrameRoadData;
    public MsgFrameBuffer msgFrameBuffer, msgFrameRoadBuffer;
    public PrimitiveList.Float.Builder K;
    public int frameID = 0;
    public boolean recording = false;
    public Context context;
    public ParamsInterface params = ParamsInterface.getInstance();
    public Fragment lifeCycleFragment;
    int cameraType;
    CameraControl cameraControl;
    SimpleDateFormat df = new SimpleDateFormat("yyyy-MM-dd--HH-mm-ss.SSS");
    ByteBuffer yuvBuffer, yuvRoadBuffer;
    String videoFileName, vidFilePath, videoLockPath;
    File lockFile;

    public CameraSelector getCameraSelector(boolean  wide){
        if (wide) {
            List<CameraInfo> availableCamerasInfo = cameraProvider.getAvailableCameraInfos();
            android.hardware.camera2.CameraManager cameraService = (android.hardware.camera2.CameraManager) context.getSystemService(Context.CAMERA_SERVICE);

            float minFocalLen = Float.MAX_VALUE;
            String wideAngleCameraId = null;

            try {
                String[] cameraIds = cameraService.getCameraIdList();
                for (String id : cameraIds) {
                    CameraCharacteristics characteristics = cameraService.getCameraCharacteristics(id);
                    float focal_length = characteristics.get(CameraCharacteristics.LENS_INFO_AVAILABLE_FOCAL_LENGTHS)[0];
                    boolean backCamera = CameraCharacteristics.LENS_FACING_BACK == characteristics.get(CameraCharacteristics.LENS_FACING);
                    if ((focal_length < minFocalLen) && backCamera) {
                        minFocalLen = focal_length;
                        wideAngleCameraId = id;
                    }
                }
            } catch (CameraAccessException e) {
                e.printStackTrace();
            }
            if (params.exists("WideCameraID")){
                wideAngleCameraId = params.getString("WideCameraID");
                System.out.println("Using camera ID provided by 'WideCameraID' param, ID: " + wideAngleCameraId);
            }
            return availableCamerasInfo.get(Integer.parseInt(wideAngleCameraId)).getCameraSelector();
        }
        else
            return new CameraSelector.Builder().requireLensFacing(CameraSelector.LENS_FACING_BACK).build();
    }

    public CameraManager(Context context, int cameraType){
        msgFrameData = new MsgFrameData(cameraType);
        K = msgFrameData.intrinsics;
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        df.setTimeZone(TimeZone.getTimeZone("UTC"));
        this.context = context;
        this.cameraType = cameraType;
        this.threadpool = Executors.newSingleThreadExecutor();

        if (cameraType == Camera.CAMERA_TYPE_WIDE){
            this.frameDataTopic = "wideRoadCameraState";
            this.frameBufferTopic = "wideRoadCameraBuffer";
            this.intName = "WideCameraMatrix";
        } else if (cameraType == CAMERA_TYPE_ROAD) {
            this.frameDataTopic = "roadCameraState";
            this.frameBufferTopic = "roadCameraBuffer";
            this.intName = "CameraMatrix";
        }

        msgFrameBuffer = new MsgFrameBuffer(W * H * 3/2, cameraType);
        yuvBuffer = msgFrameBuffer.frameBuffer.getImage().asByteBuffer();
        msgFrameBuffer.frameBuffer.setEncoding(Definitions.FrameBuffer.Encoding.YUV);
        msgFrameBuffer.frameBuffer.setFrameHeight(H);
        msgFrameBuffer.frameBuffer.setFrameWidth(W);

        ph = new ZMQPubHandler();

        ph.createPublishers(Arrays.asList(frameDataTopic, frameBufferTopic));

        loadIntrinsics();
    }

    public void loadIntrinsics(){
        if (params.exists(intName)) {
            float[] cameraMatrix = byteToFloat(params.getBytes(intName));
            updateProperty("intrinsics", cameraMatrix);
            ModelExecutorF3.updateCameraMatrix(cameraMatrix, cameraType == CAMERA_TYPE_WIDE);
            if (utils.WideCameraOnly) ModelExecutorF3.updateCameraMatrix(cameraMatrix, false);
        }
    }

    public void setIntrinsics(float[] intrinsics){
        K.set(0, intrinsics[0]);
        K.set(2, intrinsics[2]);
        K.set(4,intrinsics[4]);
        K.set(5, intrinsics[5]);
        K.set(8, 1f);
    }

    public void setLifeCycleFragment(Fragment lifeCycleFragment){
        this.lifeCycleFragment = lifeCycleFragment;
    }

    public void start() {
        if (running)
            return;
        running = true;

        CameraManager myCamManager = this;
        ListenableFuture<ProcessCameraProvider> cameraProviderFuture = ProcessCameraProvider.getInstance(context);
        cameraProviderFuture.addListener(new Runnable() {
            @Override
            public void run() {
                try {
                    cameraProvider = cameraProviderFuture.get();
                    myAnalyzer = new ImageAnalysis.Analyzer() {
                        @OptIn(markerClass = ExperimentalGetImage.class) @RequiresApi(api = Build.VERSION_CODES.N)
                        @Override
                        public void analyze(@NonNull ImageProxy image) {
                            fillYUVBuffer(image, yuvBuffer);

                            ImageProxy.PlaneProxy yPlane = image.getPlanes()[0];

                            msgFrameBuffer.frameBuffer.setYWidth(W);
                            msgFrameBuffer.frameBuffer.setYHeight(H);
                            msgFrameBuffer.frameBuffer.setYPixelStride(yPlane.getPixelStride());
                            msgFrameBuffer.frameBuffer.setUvWidth(W /2);
                            msgFrameBuffer.frameBuffer.setUvHeight(H /2);
                            msgFrameBuffer.frameBuffer.setUvPixelStride(image.getPlanes()[1].getPixelStride());
                            msgFrameBuffer.frameBuffer.setUOffset(W * H);
                            if (image.getPlanes()[1].getPixelStride() == 2)
                                msgFrameBuffer.frameBuffer.setVOffset(W * H +1);
                            else
                                msgFrameBuffer.frameBuffer.setVOffset(W * H + W * H /4);
                            msgFrameBuffer.frameBuffer.setStride(yPlane.getRowStride());

                            msgFrameData.frameData.setFrameId(frameID);

                            ModelExecutorF3.instance.ExecuteModel(msgFrameData.frameData.asReader(),
                                                                  msgFrameBuffer.frameBuffer.asReader(),
                                                                  image.getImageInfo().getTimestamp());

                            // all done, you can give us another image
                            frameID += 1;
                            image.close();

                            // do this later, don't hold up the image analyzer
                            threadpool.submit(() -> {
                                ph.publishBuffer(frameDataTopic, msgFrameData.serialize(true));
                                ph.publishBuffer(frameBufferTopic, msgFrameBuffer.serialize(true));
                            });
                        }
                    };

                    //if (utils.WideCameraOnly)
                    bindUseCases(cameraProvider);
                    /*else {
                        Managers.add(myCamManager);
                        if (Managers.size() == 2)
                            bindUseCasesGroup(Managers, cameraProvider);
                    }*/
                } catch (ExecutionException | InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }, ContextCompat.getMainExecutor(context));
    }

    /*@SuppressLint({"RestrictedApi", "UnsafeOptInUsageError"})
    private static void bindUseCasesGroup(List<CameraManager> managers, ProcessCameraProvider cameraProvider) {
        List<ConcurrentCamera.SingleCameraConfig> configs = new ArrayList<>();
        for (int i=0; i<managers.size(); i++) {
            CameraManager cm = managers.get(i);
            ImageAnalysis.Builder builder = new ImageAnalysis.Builder();
            builder.setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST);
            builder.setDefaultResolution(new Size(cm.W, cm.H));
            builder.setMaxResolution(new Size(cm.W, cm.H));
            builder.setTargetResolution(new Size(cm.W, cm.H));
            Camera2Interop.Extender<ImageAnalysis> ext = new Camera2Interop.Extender<>(builder);
            ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_REGIONS, new MeteringRectangle[]{
                    new MeteringRectangle(1, 1, cm.W - 2, cm.H - 2, 500)
            });
            ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(21, 40));
            ImageAnalysis imageAnalysis = builder.build();
            imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(cm.context), cm.myAnalyzer);
            ConcurrentCamera.SingleCameraConfig camconfig = new ConcurrentCamera.SingleCameraConfig(
                    cm.getCameraSelector(cm.cameraType == Camera.CAMERA_TYPE_WIDE),
                    new UseCaseGroup.Builder()
                            .addUseCase(imageAnalysis)
                            .build(),
                    cm.lifeCycleFragment.getViewLifecycleOwner());
            configs.add(camconfig);
        }
        ConcurrentCamera concurrentCamera = cameraProvider.bindToLifecycle(configs);
        List<androidx.camera.core.Camera> cams = concurrentCamera.getCameras();
        for (int i=0; i<cams.size(); i++) {
            CameraControl cc = cams.get(i).getCameraControl();
            cc.cancelFocusAndMetering();
        }
    }*/

    @SuppressLint({"RestrictedApi", "UnsafeOptInUsageError"})
    private void bindUseCases(@NonNull ProcessCameraProvider cameraProvider) {
        ImageAnalysis.Builder builder = new ImageAnalysis.Builder();
        builder.setBackpressureStrategy(ImageAnalysis.STRATEGY_KEEP_ONLY_LATEST);
        Size ims = new Size(W, H);
        builder.setDefaultResolution(ims);
        builder.setMaxResolution(ims);
        builder.setTargetResolution(ims);
        Camera2Interop.Extender<ImageAnalysis> ext = new Camera2Interop.Extender<>(builder);
        // try to box just the road area for metering
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_REGIONS, new MeteringRectangle[]{
                new MeteringRectangle((int)Math.floor(W * 0.05f), (int)Math.floor(H * 0.25f),
                                      (int)Math.floor(W * 0.9f),  (int)Math.floor(H * 0.70f), 500)
        });
        /*float[] lowerContrastCurve = new float[] {
                0f,   1f/8f,
                1/8f, 4f/8f,
                1f,   7f/8f,
        };
        TonemapCurve curve = new TonemapCurve(lowerContrastCurve, lowerContrastCurve, lowerContrastCurve);
        ext.setCaptureRequestOption(CaptureRequest.TONEMAP_MODE, CameraMetadata.TONEMAP_MODE_CONTRAST_CURVE);
        ext.setCaptureRequestOption(CaptureRequest.TONEMAP_CURVE, curve);*/
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(20, 20));
        ImageAnalysis imageAnalysis = builder.build();
        imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(context), myAnalyzer);

        // f3 uses wide camera.
        CameraSelector cameraSelector = getCameraSelector(cameraType == Camera.CAMERA_TYPE_WIDE);

        androidx.camera.core.Camera camera = cameraProvider.bindToLifecycle(lifeCycleFragment.getViewLifecycleOwner(), cameraSelector,
                imageAnalysis);

        cameraControl = camera.getCameraControl();

        // disable autofocus
        cameraControl.cancelFocusAndMetering();
    }

    @Override
    public void updateProperty(String property, float[] value) {
        if (property.equals("intrinsics")){
            assert value.length == 9 : "invalid intrinsic matrix buffer length";
            setIntrinsics(value);
        }
    }

    public boolean isRunning() {
        return running;
    }

    @SuppressLint("RestrictedApi")
    @Override
    public void stop() {
        // TODO: add pause/resume functionality
        if (!running)
            return;
        cameraProvider.unbindAll();
        running = false;
    }

    @Override
    public void dispose(){
        stop();
        ph.releaseAll();
    }
}
