package ai.flow.modeld;

import ai.flow.common.ParamsInterface;
import ai.flow.common.transformations.Camera;
import ai.flow.common.utils;
import ai.flow.definitions.Definitions;
import ai.flow.modeld.messages.MsgModelRaw;
import messaging.ZMQPubHandler;
import messaging.ZMQSubHandler;
import org.capnproto.PrimitiveList;
import org.nd4j.linalg.api.buffer.DataBuffer;
import org.nd4j.linalg.api.buffer.DataType;
import org.nd4j.linalg.api.buffer.util.DataTypeUtil;
import org.nd4j.linalg.api.memory.MemoryWorkspace;
import org.nd4j.linalg.api.memory.conf.WorkspaceConfiguration;
import org.nd4j.linalg.api.memory.enums.AllocationPolicy;
import org.nd4j.linalg.api.memory.enums.LearningPolicy;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.indexing.INDArrayIndex;
import org.nd4j.linalg.indexing.NDArrayIndex;
import org.opencv.core.Core;

import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import static ai.flow.common.SystemUtils.getUseGPU;
import static ai.flow.common.utils.numElements;
import static ai.flow.sensor.messages.MsgFrameBuffer.updateImageBuffer;

public class ModelExecutorF3 extends ModelExecutor {

    public boolean stopped = true;
    public boolean initialized = false;
    public long timePerIt = 0;
    public static long AvgIterationTime = 0;
    public long iterationNum = 1;

    public INDArrayIndex[] featureRotateSlice0;
    public INDArrayIndex[] featureRotateSlice1;

    public static int[] imgTensorShape = {1, 12, 128, 256};
    public static int[] featureTensorShape;
    public static final int[] trafficTensorShape = {1, CommonModelF3.TRAFFIC_CONVENTION_LEN};
    public static final int[] outputTensorShape = {1, CommonModelF3.NET_OUTPUT_SIZE};

    public static final Map<String, int[]> inputShapeMap = new HashMap<>();
    public static final Map<String, int[]> outputShapeMap = new HashMap<>();
    public INDArray trafficNDArr;
    public INDArray featuresNDArr;
    public final float[] netOutputs = new float[(int)numElements(outputTensorShape)];
    public final Map<String, INDArray> inputMap =  new HashMap<>();
    public final Map<String, float[]> outputMap =  new HashMap<>();

    public final ParamsInterface params = ParamsInterface.getInstance();

    public static final int[] FULL_FRAME_SIZE = Camera.frameSize;
    public final ZMQPubHandler ph = new ZMQPubHandler();
    public final ZMQSubHandler sh = new ZMQSubHandler(true);
    public MsgModelRaw msgModelRaw = new MsgModelRaw(CommonModelF3.NET_OUTPUT_SIZE);
    public Definitions.LiveCalibrationData.Reader liveCalib;

    public long start, end;
    public int lastFrameID = 0;

    public static int desire;
    public ModelRunner modelRunner;
    ByteBuffer imgBuffer;
    ByteBuffer wideImgBuffer;
    final WorkspaceConfiguration wsConfig = WorkspaceConfiguration.builder()
            .policyAllocation(AllocationPolicy.STRICT)
            .policyLearning(LearningPolicy.FIRST_LOOP)
            .build();

    public ModelExecutorF3(ModelRunner modelRunner){
        this.modelRunner = modelRunner;
        instance = this;
    }

    public static boolean isIntrinsicsValid(float[] intrinsics){
        // TODO: find better ways to check validity.
        return intrinsics[0]!=0 & intrinsics[2]!=0 & intrinsics[4]!=0 & intrinsics[5]!=0 & intrinsics[8]!=0;
    }

    INDArray netInputBuffer, netInputWideBuffer;
    INDArray wrapMatrix;
    INDArray wrapMatrixWide;
    ImagePrepare imagePrepare;
    ImagePrepare imageWidePrepare;

    public void ExecuteModel(Definitions.FrameData.Reader wideData, Definitions.FrameBuffer.Reader wideBuf,
                             long processStartTimestamp) {
        frameWideData = frameData = wideData;
        msgFrameWideBuffer = msgFrameBuffer = wideBuf;

        if (stopped || initialized == false) return;

        start = System.currentTimeMillis();
        imgBuffer = updateImageBuffer(msgFrameBuffer, imgBuffer);
        wideImgBuffer = updateImageBuffer(msgFrameWideBuffer, wideImgBuffer);

        if (sh.updated("lateralPlan")){
            desire = sh.recv("lateralPlan").getLateralPlan().getDesire().ordinal();
        }

        if (sh.updated("liveCalibration")) {
            liveCalib = sh.recv("liveCalibration").getLiveCalibration();
            PrimitiveList.Float.Reader rpy = liveCalib.getRpyCalib();
            for (int i = 0; i < 3; i++) {
                rpy_calib.putScalar(i, rpy.get(i));
            }
            wrapMatrix = Preprocess.getWrapMatrix(rpy_calib, Camera.cam_intrinsics, Camera.cam_intrinsics, true, false);
            wrapMatrixWide = Preprocess.getWrapMatrix(rpy_calib, Camera.cam_intrinsics, Camera.cam_intrinsics, true, true);
        }

        netInputBuffer = imagePrepare.prepare(imgBuffer, wrapMatrix);
        netInputWideBuffer = imageWidePrepare.prepare(wideImgBuffer, wrapMatrixWide);

        inputMap.put("input_imgs", netInputBuffer);
        inputMap.put("big_input_imgs", netInputWideBuffer);
        modelRunner.run(inputMap, outputMap);

        // publish outputs
        end = System.currentTimeMillis();
        msgModelRaw.fill(netOutputs, processStartTimestamp, lastFrameID, 0, 0f, end - start);
        ph.publishBuffer("modelRaw", msgModelRaw.serialize(true));

        // compute runtime stats every 10 runs
        timePerIt += end - processStartTimestamp;
        iterationNum++;
        if (iterationNum > 10) {
            AvgIterationTime = timePerIt / iterationNum;
            iterationNum = 0;
            timePerIt = 0;
        }

        lastFrameID = frameData.getFrameId();
    }

    public void init(){
        if (initialized) return;
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        featureRotateSlice0 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(0, CommonModelF3.HISTORY_BUFFER_LEN-1), NDArrayIndex.all() };
        featureRotateSlice1 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(1, CommonModelF3.HISTORY_BUFFER_LEN), NDArrayIndex.all() };
        featureTensorShape = new int[] {1, CommonModelF3.HISTORY_BUFFER_LEN, CommonModelF3.FEATURE_LEN };

        trafficNDArr = Nd4j.zeros(trafficTensorShape);
        featuresNDArr = Nd4j.zeros(featureTensorShape);

        ph.createPublishers(Arrays.asList("modelRaw"));
        sh.createSubscribers(Arrays.asList("pulseDesire", "liveCalibration", "lateralPlan"));

        inputShapeMap.put("input_imgs", imgTensorShape);
        inputShapeMap.put("big_input_imgs", imgTensorShape);
        inputShapeMap.put("features_buffer", featureTensorShape);
        inputShapeMap.put("traffic_convention", trafficTensorShape);
        outputShapeMap.put("outputs", outputTensorShape);
        inputMap.put("features_buffer", featuresNDArr);
        inputMap.put("traffic_convention", trafficNDArr);
        outputMap.put("outputs", netOutputs);

        modelRunner.init(inputShapeMap, outputShapeMap);
        modelRunner.warmup();

        wrapMatrix = Preprocess.getWrapMatrix(rpy_calib, Camera.cam_intrinsics, Camera.cam_intrinsics, true, false);
        wrapMatrixWide = Preprocess.getWrapMatrix(rpy_calib, Camera.cam_intrinsics, Camera.cam_intrinsics, true, true);

        // wait for a frame
        while (msgFrameBuffer == null) {
            try {
                Thread.sleep(10);
            } catch (Exception e) {}
        }

        // TODO:Clean this shit.
        boolean rgb;
        if (getUseGPU()){
            rgb = msgFrameBuffer.getEncoding() == Definitions.FrameBuffer.Encoding.RGB;
            imagePrepare = new ImagePrepareGPU(FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1], rgb, msgFrameBuffer.getYWidth(), msgFrameBuffer.getYHeight(),
                    msgFrameBuffer.getYPixelStride(), msgFrameBuffer.getUvWidth(), msgFrameBuffer.getUvHeight(), msgFrameBuffer.getUvPixelStride(),
                    msgFrameBuffer.getUOffset(), msgFrameBuffer.getVOffset(), msgFrameBuffer.getStride());
            rgb = msgFrameWideBuffer.getEncoding() == Definitions.FrameBuffer.Encoding.RGB;
            imageWidePrepare = new ImagePrepareGPU(FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1], rgb, msgFrameWideBuffer.getYWidth(), msgFrameWideBuffer.getYHeight(),
                    msgFrameWideBuffer.getYPixelStride(), msgFrameWideBuffer.getUvWidth(), msgFrameWideBuffer.getUvHeight(), msgFrameWideBuffer.getUvPixelStride(),
                    msgFrameWideBuffer.getUOffset(), msgFrameWideBuffer.getVOffset(), msgFrameWideBuffer.getStride());
        }
        else{
            rgb = msgFrameBuffer.getEncoding() == Definitions.FrameBuffer.Encoding.RGB;
            imagePrepare = new ImagePrepareCPU(FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1], rgb, msgFrameBuffer.getYWidth(), msgFrameBuffer.getYHeight(),
                    msgFrameBuffer.getYPixelStride(), msgFrameBuffer.getUvWidth(), msgFrameBuffer.getUvHeight(), msgFrameBuffer.getUvPixelStride(),
                    msgFrameBuffer.getUOffset(), msgFrameBuffer.getVOffset(), msgFrameBuffer.getStride());
            rgb = msgFrameWideBuffer.getEncoding() == Definitions.FrameBuffer.Encoding.RGB;
            imageWidePrepare = new ImagePrepareCPU(FULL_FRAME_SIZE[0], FULL_FRAME_SIZE[1], rgb, msgFrameWideBuffer.getYWidth(), msgFrameWideBuffer.getYHeight(),
                    msgFrameWideBuffer.getYPixelStride(), msgFrameWideBuffer.getUvWidth(), msgFrameWideBuffer.getUvHeight(), msgFrameWideBuffer.getUvPixelStride(),
                    msgFrameWideBuffer.getUOffset(), msgFrameWideBuffer.getVOffset(), msgFrameWideBuffer.getStride());
        }

        initialized = true;
        params.putBool("ModelDReady", true);
    }

    public long getIterationRate() {
        return timePerIt/iterationNum;
    }

    public boolean isRunning() {
        return !stopped;
    }

    public boolean isInitialized(){
        return initialized;
    }

    public void dispose(){
        // dispose
        stopped = true;
        wrapMatrix.close();
        wrapMatrixWide.close();

        for (String inputName : inputMap.keySet()) {
            inputMap.get(inputName).close();
        }
        modelRunner.dispose();
        imagePrepare.dispose();
        imageWidePrepare.dispose();
        ph.releaseAll();
    }

    public void stop() {
        stopped = true;
    }

    public void start(){
        stopped = false;
    }
}
