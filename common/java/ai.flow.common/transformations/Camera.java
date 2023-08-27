package ai.flow.common.transformations;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

public class Camera {

    // lots of important stuff calculated from this (Camera #2)
    public static final float[] WideIntrinsics = {
            1394.7081f, 0.0f, 952.62915f,
            0.0f,   1394.7616f, 517.53534f,
            0.0f,   0.0f, 1.0f
    };

    // Camera #0 (telephoto)
    /*public static final float[] WideIntrinsics = {
            910f,   0.0f, 900f,
            0.0f,   910f, 514f,
            0.0f,   0.0f, 1.0f
    };*/

    // everything auto-generated from above
    public static final float eon_wide_focal_length = 910.0f; // model expects this for some reason
    public static final float actual_cam_intrinsics = (WideIntrinsics[0] + WideIntrinsics[4]) * 0.5f;
    public static final float digital_zoom_apply = actual_cam_intrinsics / eon_wide_focal_length;

    public static final int CAMERA_TYPE_ROAD = 0;
    public static final int CAMERA_TYPE_WIDE = 1;
    public static final int CAMERA_TYPE_DRIVER = 2;
    public static final int[] frameSize = new int[]{1920, 1080};
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
