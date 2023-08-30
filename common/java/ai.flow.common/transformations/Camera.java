package ai.flow.common.transformations;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

public class Camera {

    // lots of important stuff calculated from this
    // current set to LG G8 wide camera
    public static final float
        FocalX = 1600f, //1394.7081f,
        FocalY = 1600f, //1394.7616f,
        CenterX = 952.62915f,
        CenterY = 517.53534f;

    // everything autocalculated below
    public static final float eon_wide_focal_length = 910.0f; // model expects this for some reason
    public static final float actual_cam_focal_length = (FocalX + FocalY) * 0.5f;
    public static final float digital_zoom_apply = actual_cam_focal_length / eon_wide_focal_length;
    public static final int[] frameSize = new int[]{1920, 1080};
    public static final float OffsetX = CenterX - (frameSize[0]*0.5f);
    public static final float OffsetY = CenterY - (frameSize[1]*0.5f);

    public static final float[] WideIntrinsics = {
            FocalX, 0.0f, frameSize[0] * 0.5f + OffsetX * digital_zoom_apply,
            0.0f, FocalY, frameSize[1] * 0.5f + OffsetY * digital_zoom_apply,
            0.0f,   0.0f, 1.0f
    };

    // everything auto-generated from above
    public static final int CAMERA_TYPE_ROAD = 0;
    public static final int CAMERA_TYPE_WIDE = 1;
    public static final int CAMERA_TYPE_DRIVER = 2;
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

    // Camera #0 (telephoto)
    /*public static final float[] WideIntrinsics = {
            910f,   0.0f, 900f,
            0.0f,   910f, 514f,
            0.0f,   0.0f, 1.0f
    };*/
}
