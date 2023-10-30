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
    public INDArrayIndex[] desireFeatureSlice0;
    public INDArrayIndex[] desireFeatureSlice1;

    public static int[] imgTensorShape = {1, 12, 128, 256};
    public static int[] featureTensorShape;
    public static int[] desireTensorShape;
    public static final int[] trafficTensorShape = {1, CommonModelF3.TRAFFIC_CONVENTION_LEN};
    public static final int[] outputTensorShape = {1, CommonModelF3.NET_OUTPUT_SIZE};
    public static final int[] navFeaturesTensorShape = {1, CommonModelF3.NAV_FEATURE_LEN};
    public static final int[] navInstructionsTensorShape = {1, 150};

    public static final Map<String, int[]> inputShapeMap = new HashMap<>();
    public static final Map<String, int[]> outputShapeMap = new HashMap<>();
    public INDArray desireNDArr;
    public INDArray trafficNDArr;
    public INDArray featuresNDArr;
    public INDArray navfeaturesNDArr;
    public INDArray navinstructNDArr;
    public final float[] netOutputs = new float[(int)numElements(outputTensorShape)];
    public final INDArray augmentRot = Nd4j.zeros(3);
    public final INDArray augmentTrans = Nd4j.zeros(3);
    public final float[]prevDesire = new float[CommonModelF3.DESIRE_LEN];
    public final float[]desireIn = new float[CommonModelF3.DESIRE_LEN];
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

    int desire;
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
            for (int i=0; i<CommonModelF3.DESIRE_LEN; i++)
                desireIn[i] = i == desire ? 1f : 0f;
        }

        //std::memmove(&s->pulse_desire[0], &s->pulse_desire[DESIRE_LEN], sizeof(float) * DESIRE_LEN*HISTORY_BUFFER_LEN);
        desireNDArr.put(desireFeatureSlice0, desireNDArr.get(desireFeatureSlice1));
        for (int i=1; i<CommonModelF3.DESIRE_LEN; i++){
            if (desireIn[i] - prevDesire[i] > 0.99f) {
                if (utils.Runner == utils.USE_MODEL_RUNNER.SNPE)
                    desireNDArr.putScalar(0, i, CommonModelF3.HISTORY_BUFFER_LEN, desireIn[i]);
                else
                    desireNDArr.putScalar(0, CommonModelF3.HISTORY_BUFFER_LEN, i, desireIn[i]);
            } else {
                if (utils.Runner == utils.USE_MODEL_RUNNER.SNPE)
                    desireNDArr.putScalar(0, i, CommonModelF3.HISTORY_BUFFER_LEN, 0.0f);
                else
                    desireNDArr.putScalar(0, CommonModelF3.HISTORY_BUFFER_LEN, i,0.0f);
            }
            prevDesire[i] = desireIn[i];
        }

        if (sh.updated("liveCalibration")) {
            liveCalib = sh.recv("liveCalibration").getLiveCalibration();
            PrimitiveList.Float.Reader rpy = liveCalib.getRpyCalib();
            for (int i = 0; i < 3; i++) {
                augmentRot.putScalar(i, rpy.get(i));
            }
            wrapMatrix = Preprocess.getWrapMatrix(augmentRot, Camera.cam_intrinsics, Camera.cam_intrinsics, true, false);
            wrapMatrixWide = Preprocess.getWrapMatrix(augmentRot, Camera.cam_intrinsics, Camera.cam_intrinsics, true, true);
        }

        netInputBuffer = imagePrepare.prepare(imgBuffer, wrapMatrix);
        netInputWideBuffer = imageWidePrepare.prepare(wideImgBuffer, wrapMatrixWide);

        if (utils.Runner == utils.USE_MODEL_RUNNER.SNPE){
            try (MemoryWorkspace ws = Nd4j.getWorkspaceManager().getAndActivateWorkspace(wsConfig, "ModelD")) {
                // NCHW to NHWC
                netInputBuffer = netInputBuffer.permute(0, 2, 3, 1).dup();
                netInputWideBuffer = netInputWideBuffer.permute(0, 2, 3, 1).dup();
            }
        }

        inputMap.put("input_imgs", netInputBuffer);
        inputMap.put("big_input_imgs", netInputWideBuffer);
        modelRunner.run(inputMap, outputMap);

        // featureTensorShape, 1, FEATURE_LEN, HISTORY_LEN
        featuresNDArr.put(featureRotateSlice0, featuresNDArr.get(featureRotateSlice1));
        if (utils.Runner == utils.USE_MODEL_RUNNER.SNPE) {
            for (int i = 0; i < CommonModelF3.FEATURE_LEN; i++)
                featuresNDArr.putScalar(0, i,CommonModelF3.HISTORY_BUFFER_LEN - 1, netOutputs[CommonModelF3.OUTPUT_SIZE + i]); // SNPE
        } else {
            for (int i = 0; i < CommonModelF3.FEATURE_LEN; i++)
                featuresNDArr.putScalar(0, CommonModelF3.HISTORY_BUFFER_LEN - 1, i, netOutputs[CommonModelF3.OUTPUT_SIZE + i]);
        }

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

        if (utils.Runner == utils.USE_MODEL_RUNNER.SNPE) {
            imgTensorShape = new int[]{1, 128, 256, 12};
            featureRotateSlice0 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.all(), NDArrayIndex.interval(0, CommonModelF3.HISTORY_BUFFER_LEN-1), }; // SNPE
            featureRotateSlice1 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.all(), NDArrayIndex.interval(1, CommonModelF3.HISTORY_BUFFER_LEN) };
            desireFeatureSlice0 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.all(), NDArrayIndex.interval(0, CommonModelF3.HISTORY_BUFFER_LEN) };
            desireFeatureSlice1 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.all(), NDArrayIndex.interval(1, CommonModelF3.HISTORY_BUFFER_LEN+1) };
            featureTensorShape = new int[] {1, CommonModelF3.FEATURE_LEN, CommonModelF3.HISTORY_BUFFER_LEN};
            desireTensorShape = new int[] {1, CommonModelF3.DESIRE_LEN, CommonModelF3.HISTORY_BUFFER_LEN+1};
        } else {
            featureRotateSlice0 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(0, CommonModelF3.HISTORY_BUFFER_LEN-1), NDArrayIndex.all() };
            featureRotateSlice1 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(1, CommonModelF3.HISTORY_BUFFER_LEN), NDArrayIndex.all() };
            desireFeatureSlice0 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(0, CommonModelF3.HISTORY_BUFFER_LEN), NDArrayIndex.all() };
            desireFeatureSlice1 = new INDArrayIndex[]{NDArrayIndex.point(0), NDArrayIndex.interval(1, CommonModelF3.HISTORY_BUFFER_LEN+1), NDArrayIndex.all() };
            featureTensorShape = new int[] {1, CommonModelF3.HISTORY_BUFFER_LEN, CommonModelF3.FEATURE_LEN };
            desireTensorShape = new int[] {1, CommonModelF3.HISTORY_BUFFER_LEN+1, CommonModelF3.DESIRE_LEN };
        }

        desireNDArr = Nd4j.zeros(desireTensorShape);
        trafficNDArr = Nd4j.zeros(trafficTensorShape);
        featuresNDArr = Nd4j.zeros(featureTensorShape);
        navfeaturesNDArr = Nd4j.zeros(navFeaturesTensorShape);
        navinstructNDArr = Nd4j.zeros(navInstructionsTensorShape);

        ph.createPublishers(Arrays.asList("modelRaw"));
        sh.createSubscribers(Arrays.asList("pulseDesire", "liveCalibration", "lateralPlan"));

        inputShapeMap.put("input_imgs", imgTensorShape);
        inputShapeMap.put("big_input_imgs", imgTensorShape);
        inputShapeMap.put("features_buffer", featureTensorShape);
        inputShapeMap.put("desire", desireTensorShape);
        inputShapeMap.put("traffic_convention", trafficTensorShape);
        inputShapeMap.put("nav_features", navFeaturesTensorShape);
        if (utils.Runner != utils.USE_MODEL_RUNNER.SNPE)
            inputShapeMap.put("nav_instructions", navInstructionsTensorShape);
        outputShapeMap.put("outputs", outputTensorShape);

        inputMap.put("nav_features", navfeaturesNDArr);
        if (utils.Runner != utils.USE_MODEL_RUNNER.SNPE)
            inputMap.put("nav_instructions", navinstructNDArr);
        inputMap.put("features_buffer", featuresNDArr);
        inputMap.put("desire", desireNDArr);
        inputMap.put("traffic_convention", trafficNDArr);
        outputMap.put("outputs", netOutputs);

        modelRunner.init(inputShapeMap, outputShapeMap);
        modelRunner.warmup();

        wrapMatrix = Preprocess.getWrapMatrix(augmentRot, Camera.cam_intrinsics, Camera.cam_intrinsics, true, false);
        wrapMatrixWide = Preprocess.getWrapMatrix(augmentRot, Camera.cam_intrinsics, Camera.cam_intrinsics, true, true);

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
