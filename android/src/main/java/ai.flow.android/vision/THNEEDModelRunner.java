package ai.flow.android.vision;

import android.app.Application;

import org.nd4j.linalg.api.ndarray.INDArray;

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
    public static native float[] executeModel(float[] input_imgs,
                                              float[] big_input_imgs,
                                              float[] features_buffer,
                                              float[] desire,
                                              float[] traffic_convention,
                                              float[] nav_features,
                                              float[] nav_instructions);

    public THNEEDModelRunner(String modelPath, Application context){
        this.modelPath = modelPath + ".thneed";
        this.context = context;
    }

    @Override
    public void init(Map<String, int[]> shapes, Map<String, int[]> outputShapes) {
        //String libPath = context.getApplicationInfo().nativeLibraryDir;
        System.loadLibrary("thneed");
        System.loadLibrary("jniconvert");

        createStdString(modelPath);
        getArray(CommonModelF3.NET_OUTPUT_SIZE);
        initThneed();
    }

    @Override
    public void run(Map<String, INDArray> inputMap, Map<String, float[]> outputMap) {
        float[] netOutputs = executeModel(inputMap.get("input_imgs").data().asFloat(),
                                          inputMap.get("big_input_imgs").data().asFloat(),
                                          inputMap.get("features_buffer").data().asFloat(),
                                          inputMap.get("desire").data().asFloat(),
                                          inputMap.get("traffic_convention").data().asFloat(),
                                          inputMap.get("nav_features").data().asFloat(),
                                          inputMap.get("nav_instructions").data().asFloat());
        System.arraycopy(netOutputs, 0, outputMap.get("outputs"), 0, outputMap.get("outputs").length);
    }

    @Override
    public void dispose(){

    }
}
