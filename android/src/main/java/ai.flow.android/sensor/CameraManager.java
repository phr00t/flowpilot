package ai.flow.android.sensor;

import ai.flow.app.OnRoadScreen;
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
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.graphics.Rect;
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

import io.github.crow_misia.libyuv.AbgrBuffer;
import io.github.crow_misia.libyuv.FilterMode;
import io.github.crow_misia.libyuv.Nv21Buffer;
import messaging.ZMQPubHandler;
import org.capnproto.PrimitiveList;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
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
    ByteBuffer yuvBuffer;

    public CameraSelector getCameraSelector(boolean  wide){
        if (wide) {
            List<CameraInfo> availableCamerasInfo = cameraProvider.getAvailableCameraInfos();
            OnRoadScreen.CamSelected = 2;
            return availableCamerasInfo.get(OnRoadScreen.CamSelected).getCameraSelector();
        }
        else
            return new CameraSelector.Builder().requireLensFacing(CameraSelector.LENS_FACING_BACK).build();
    }

    public CameraManager(Context context, int cameraType){
        msgFrameData = new MsgFrameData(cameraType);
        K = msgFrameData.intrinsics;
        for (int i=0; i<9; i++)
            K.set(i, Camera.WideIntrinsics[i]);
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        df.setTimeZone(TimeZone.getTimeZone("UTC"));
        this.context = context;
        this.cameraType = cameraType;
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
                            long startTimestamp = System.currentTimeMillis();
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

                            ModelExecutorF3.instance.ExecuteModel(
                                    msgFrameData.frameData.asReader(),
                                    msgFrameBuffer.frameBuffer.asReader(),
                                    image.getImageInfo().getTimestamp(), startTimestamp);

                            ph.publishBuffer(frameDataTopic, msgFrameData.serialize(true));
                            ph.publishBuffer(frameBufferTopic, msgFrameBuffer.serialize(true));

                            // make sure we keep our zoom level
                            if (frameID % 5 == 0)
                                cameraControl.setZoomRatio(Camera.digital_zoom_apply);

                            frameID += 1;
                            image.close();
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
        float[] gammaCurve = new float[] {
                0.0000f, 0.0000f, 0.0667f, 0.2864f, 0.1333f, 0.4007f, 0.2000f, 0.4845f,
                0.2667f, 0.5532f, 0.3333f, 0.6125f, 0.4000f, 0.6652f, 0.4667f, 0.7130f,
                0.5333f, 0.7569f, 0.6000f, 0.7977f, 0.6667f, 0.8360f, 0.7333f, 0.8721f,
                0.8000f, 0.9063f, 0.8667f, 0.9389f, 0.9333f, 0.9701f, 1.0000f, 1.0000f
        };
        TonemapCurve curve = new TonemapCurve(gammaCurve, gammaCurve, gammaCurve);
        ext.setCaptureRequestOption(CaptureRequest.TONEMAP_MODE, CameraMetadata.TONEMAP_MODE_CONTRAST_CURVE);
        ext.setCaptureRequestOption(CaptureRequest.TONEMAP_CURVE, curve);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_MODE, CaptureRequest.CONTROL_MODE_AUTO);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_MODE, CaptureRequest.CONTROL_AE_MODE_ON);
        ext.setCaptureRequestOption(CaptureRequest.COLOR_CORRECTION_MODE, CaptureRequest.COLOR_CORRECTION_MODE_FAST);
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AE_TARGET_FPS_RANGE, new Range<>(20, 20));
        ext.setCaptureRequestOption(CaptureRequest.CONTROL_AF_MODE, CaptureRequest.CONTROL_AF_MODE_OFF);
        ext.setCaptureRequestOption(CaptureRequest.LENS_FOCUS_DISTANCE, 0f);
        ImageAnalysis imageAnalysis = builder.build();
        imageAnalysis.setAnalyzer(ContextCompat.getMainExecutor(context), myAnalyzer);

        // f3 uses wide camera.
        CameraSelector cameraSelector = getCameraSelector(cameraType == Camera.CAMERA_TYPE_WIDE);

        androidx.camera.core.Camera camera = cameraProvider.bindToLifecycle(lifeCycleFragment.getViewLifecycleOwner(), cameraSelector,
                imageAnalysis);

        cameraControl = camera.getCameraControl();
        cameraControl.setZoomRatio(Camera.digital_zoom_apply);
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
