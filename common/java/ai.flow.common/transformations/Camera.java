package ai.flow.common.transformations;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

public class Camera {

    public static final int CAMERA_TYPE_ROAD = 0;
    public static final int CAMERA_TYPE_WIDE = 1;
    public static final int CAMERA_TYPE_DRIVER = 2;
    public static final int[] frameSize = new int[]{1920, 1080}; // : new int[]{1280, 720};
    // we only use the wide camera. Hardcoded LG G8 wide cam intrinsics
    public static INDArray wide_intrinsics = Nd4j.createFromArray(new float[][]{
            {1394.7081f,  0.0f,  952.62915f}, // make sure we match CameraManager.K
            {0.0f,  1394.7616f,  517.53534f},
            {0.0f,  0.0f,  1.0f}
    });

    public static final INDArray view_from_device = Nd4j.createFromArray(new float[][]{
            {0.0f,  1.0f,  0.0f},
            {0.0f,  0.0f,  1.0f},
            {1.0f,  0.0f,  0.0f}
    });
}
