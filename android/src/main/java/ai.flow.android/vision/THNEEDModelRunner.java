package ai.flow.android.vision;

import android.app.Application;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import ai.flow.modeld.CommonModelF3;
import ai.flow.modeld.ModelRunner;

public class THNEEDModelRunner extends ModelRunner {

    String modelPath;
    Application context;

    // native function
    public static native void createStdString(String javaString);
    public static native void getArray(int size);
    public static native void initThneed();
    public static native float[] executeModel(float[] input);
    public float[] inputBuffer;

    public THNEEDModelRunner(String modelPath, Application context){
        this.modelPath = modelPath + ".thneed";
        this.context = context;
    }

    @Override
    public void init(Map<String, int[]> shapes, Map<String, int[]> outputShapes) {
        //String libPath = context.getApplicationInfo().nativeLibraryDir;
        //System.loadLibrary("stdc++");
        //System.loadLibrary("thneed");
        System.loadLibrary("jniconvert");

        createStdString(modelPath);
        getArray(CommonModelF3.NET_OUTPUT_SIZE);
        initThneed();
        inputBuffer = new float[(1024/4) + 2 * (1572864 / 4) + (99 * 512) + (3200 / 4)];
        // init first part of float to 0
        for (int i=0; i<1024/4; i++)
            inputBuffer[0] = 0f;
    }

    @Override
    public void run(Map<String, INDArray> inputMap, Map<String, float[]> outputMap) {
        // prepare input
        int zerolen = 1024/4;
        int img_len = 1572864 / 4;
        int feature_len = 99 * 512;
        int desire_len = 3200 / 4;
        inputMap.get("input_imgs").data().asNioFloat().get(inputBuffer, zerolen, img_len);
        inputMap.get("big_input_imgs").data().asNioFloat().get(inputBuffer, zerolen + img_len , img_len);
        inputMap.get("features_buffer").data().asNioFloat().get(inputBuffer, zerolen + img_len * 2, feature_len);
        inputMap.get("desire").data().asNioFloat().get(inputBuffer, zerolen + img_len * 2 + feature_len, desire_len);
        float[] netOutputs = executeModel(inputBuffer);
        System.arraycopy(netOutputs, 0, outputMap.get("outputs"), 0, outputMap.get("outputs").length);
    }

    @Override
    public void dispose(){

    }
}
