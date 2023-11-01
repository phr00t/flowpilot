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
    }

    @Override
    public void run(Map<String, INDArray> inputMap, Map<String, float[]> outputMap) {
        float[] inputs = Nd4j.toFlattened(Nd4j.zeros(1024/4), inputMap.get("input_imgs"), inputMap.get("big_input_imgs"), inputMap.get("features_buffer"), inputMap.get("desire"))
                .data().asFloat();
        float[] netOutputs = executeModel(inputs);
        System.arraycopy(netOutputs, 0, outputMap.get("outputs"), 0, outputMap.get("outputs").length);
    }

    @Override
    public void dispose(){

    }
}
