package ai.flow.common.transformations;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

public class Camera {

    public static final int CAMERA_TYPE_ROAD = 0;
    public static final int CAMERA_TYPE_WIDE = 1;
    public static final int CAMERA_TYPE_DRIVER = 2;
    public static final int[] frameSize = new int[]{1920, 1080}; // : new int[]{1280, 720};
    private static final float WideCamFocalLengthAdjust = 0.65f;
    // we only use the wide camera. Hardcoded LG G8 wide cam intrinsics
    public static final float[] WideIntrinsics = {
            1394.7081f * WideCamFocalLengthAdjust, 0.0f, 952.62915f,
            0.0f,   1394.7616f * WideCamFocalLengthAdjust, 517.53534f,
            0.0f,   0.0f, 1.0f
    };
    public static INDArray wide_intrinsics = Nd4j.createFromArray(new float[][]{
            { WideIntrinsics[0],  0.0f,  WideIntrinsics[2]},
            {0.0f,  WideIntrinsics[4],  WideIntrinsics[5]},
            {0.0f,  0.0f,  1.0f}
    });

    public static final INDArray view_from_device = Nd4j.createFromArray(new float[][]{
            {0.0f,  1.0f,  0.0f},
            {0.0f,  0.0f,  1.0f},
            {1.0f,  0.0f,  0.0f}
    });
}
