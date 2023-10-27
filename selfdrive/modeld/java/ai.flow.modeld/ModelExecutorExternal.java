package ai.flow.modeld;

import static ai.flow.common.SystemUtils.getUseGPU;
import static ai.flow.common.utils.numElements;
import static ai.flow.sensor.messages.MsgFrameBuffer.updateImageBuffer;

import org.capnproto.PrimitiveList;
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

import ai.flow.common.ParamsInterface;
import ai.flow.common.transformations.Camera;
import ai.flow.common.utils;
import ai.flow.definitions.Definitions;
import ai.flow.modeld.messages.MsgModelRaw;
import messaging.ZMQPubHandler;
import messaging.ZMQSubHandler;

public class ModelExecutorExternal extends ModelExecutor {

    public static ModelExecutor instance;
    public boolean stopped = true;
    public boolean initialized = false;
    public long timePerIt = 0;
    public static long AvgIterationTime = 0;
    public long iterationNum = 1;

    public static int[] imgTensorShape = {1, 12, 128, 256};

    public final INDArray augmentRot = Nd4j.zeros(3);
    public final INDArray augmentTrans = Nd4j.zeros(3);

    public final ParamsInterface params = ParamsInterface.getInstance();

    public static final int[] FULL_FRAME_SIZE = Camera.frameSize;
    public final ZMQPubHandler ph = new ZMQPubHandler();
    public final ZMQSubHandler sh = new ZMQSubHandler(true);
    public MsgModelRaw msgModelRaw = new MsgModelRaw(2 * 1572864 / 4); // 1572864 is the size of an image in bytes
    public Definitions.LiveCalibrationData.Reader liveCalib;

    public long start, end;
    public int lastFrameID = 0;

    public static Definitions.FrameData.Reader frameData;
    public static Definitions.FrameData.Reader frameWideData;
    public static Definitions.FrameBuffer.Reader msgFrameBuffer;
    public static Definitions.FrameBuffer.Reader msgFrameWideBuffer;
    ByteBuffer imgBuffer;
    ByteBuffer wideImgBuffer;
    final WorkspaceConfiguration wsConfig = WorkspaceConfiguration.builder()
            .policyAllocation(AllocationPolicy.STRICT)
            .policyLearning(LearningPolicy.FIRST_LOOP)
            .build();

    public ModelExecutorExternal(){
        instance = this;
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

        //inputMap.put("input_imgs", netInputBuffer);
        //inputMap.put("big_input_imgs", netInputWideBuffer);
        //modelRunner.run(inputMap, outputMap);

        // publish outputs
        end = System.currentTimeMillis();
        msgModelRaw.fill(Nd4j.toFlattened(netInputBuffer, netInputWideBuffer).data().asNioFloat().array(), processStartTimestamp, lastFrameID, 0, 0f, end - start);
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

        ph.createPublishers(Arrays.asList("modelRaw"));
        sh.createSubscribers(Arrays.asList("liveCalibration"));

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
