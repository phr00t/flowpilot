package ai.flow.android.vision;

import android.app.Application;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import ai.flow.app.OnRoadScreen;
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

    private final int img_len = 1572864 / 4;
    private final int desire_len = 3200 / 4;

    public THNEEDModelRunner(String modelPath, Application context){
        this.modelPath = modelPath + ".thneed";
        this.context = context;
    }

    @Override
    public void init(Map<String, int[]> shapes, Map<String, int[]> outputShapes) {
        System.loadLibrary("jniconvert");

        createStdString(modelPath);
        getArray(CommonModelF3.NET_OUTPUT_SIZE);
        initThneed();
        inputBuffer = new float[2 * (1572864 / 4) + (3200 / 4) + 2];
        // new LA model input
        inputBuffer[img_len * 2 + desire_len + 1] = 0.1f; // steering actuator delay
    }

    @Override
    public void run(Map<String, INDArray> inputMap, Map<String, float[]> outputMap) {
        // new inputs for LA
        inputBuffer[img_len * 2 + desire_len] = OnRoadScreen.LatestvEgo; // speed in m/s
        // ok regular inputs
        inputMap.get("input_imgs").data().asNioFloat().get(inputBuffer, 0, img_len);
        inputMap.get("big_input_imgs").data().asNioFloat().get(inputBuffer, img_len , img_len);
        inputMap.get("desire").data().asNioFloat().get(inputBuffer, img_len * 2, desire_len);
        float[] netOutputs = executeModel(inputBuffer);
        System.arraycopy(netOutputs, 0, outputMap.get("outputs"), 0, outputMap.get("outputs").length);
    }

    @Override
    public void dispose(){

    }
}
