package ai.flow.android.vision;

import android.app.Application;

import org.nd4j.linalg.api.ndarray.INDArray;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import ai.flow.modeld.CommonModelF3;
import ai.flow.modeld.ModelRunner;

public class THNEEDModelRunner extends ModelRunner {

    private Application context;
    String modelPath;
    Map<String, long[]> shapes = new HashMap<>();
    float[] outputs;

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
        System.loadLibrary("jniconvert");

        createStdString(modelPath);
        getArray(CommonModelF3.NET_OUTPUT_SIZE);
        initThneed();

        for (String name : shapes.keySet()) {
            this.shapes.put(name, Arrays.stream(shapes.get(name)).mapToLong((i) -> (long) i).toArray());
        }
    }

    @Override
    public void run(Map<String, INDArray> inputMap, Map<String, float[]> outputMap) {
        float[] netOutputs = executeModel(inputMap.get("input_imgs").toFloatVector(),
                                          inputMap.get("big_input_imgs").toFloatVector(),
                                          inputMap.get("features_buffer").toFloatVector(),
                                          inputMap.get("desire").toFloatVector(),
                                          inputMap.get("traffic_convention").toFloatVector(),
                                          inputMap.get("nav_features").toFloatVector(),
                                          inputMap.get("nav_instructions").toFloatVector());
        System.arraycopy(netOutputs, 0, outputMap.get("outputs"), 0, outputMap.get("outputs").length);
    }

    @Override
    public void dispose(){

    }
}
