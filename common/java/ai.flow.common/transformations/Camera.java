package ai.flow.common.transformations;

import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;

import ai.flow.common.utils;

public class Camera {

    public static final boolean FORCE_TELE_CAM_F3 = false;

    // lots of important stuff calculated from this
    // current set to LG G8 wide camera
    // NOTE: these intrinsics can be updated at runtime by updateIntrinsics()
    // (from a camerainfo file or auto-detected from the camera hardware) on the
    // camera/main thread, while the model and UI threads read them. They are
    // marked volatile so those threads reliably observe the updated values.
    public static volatile float
        FocalX = utils.F2 ? 1672.33f : (FORCE_TELE_CAM_F3 ? 910f : 1600f), //1394.7081f,
        FocalY = utils.F2 ? 1672.33f : (FORCE_TELE_CAM_F3 ? 910f : 1600f), //1394.7616f,
        CenterX = utils.F2 ? 900f : 952.62915f,
        CenterY = utils.F2 ? 514f : 517.53534f;

    public static int UseCameraID = utils.F2 ? 0 : 2;

    // set to true once the camera intrinsics have been provided explicitly
    // (e.g. via a camerainfo override file). When true, runtime auto-detection
    // of the intrinsics from the camera hardware is skipped.
    public static boolean intrinsicsLoadedFromFile = false;

    // everything autocalculated below
    public static volatile float actual_cam_focal_length = (FocalX + FocalY) * 0.5f;
    public static volatile float digital_zoom_apply = actual_cam_focal_length / (utils.F2 ? Model.MEDMODEL_F2_FL : Model.MEDMODEL_FL);
    public static final int[] frameSize = new int[]{1920, 1080};
    public static volatile float OffsetX = CenterX - (frameSize[0]*0.5f);
    public static volatile float OffsetY = CenterY - (frameSize[1]*0.5f);

    public static volatile float[] CameraIntrinsics = {
            FocalX, 0.0f, frameSize[0] * 0.5f + OffsetX * digital_zoom_apply,
            0.0f, FocalY, frameSize[1] * 0.5f + OffsetY * digital_zoom_apply,
            0.0f,   0.0f, 1.0f
    };

    // everything auto-generated from above
    public static final int CAMERA_TYPE_ROAD = 0;
    public static final int CAMERA_TYPE_WIDE = 1;
    public static final int CAMERA_TYPE_DRIVER = 2;
    public static volatile INDArray cam_intrinsics = Nd4j.createFromArray(new float[][]{
            { CameraIntrinsics[0],  0.0f,  CameraIntrinsics[2]},
            {0.0f,  CameraIntrinsics[4],  CameraIntrinsics[5]},
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

    // Recompute every derived intrinsic value from FocalX/FocalY/CenterX/CenterY.
    // Call this after changing any of those (e.g. when loading them from a file or
    // auto-detecting them from the camera hardware) so the rest of the pipeline
    // picks up the new values.
    public static void updateIntrinsics() {
        actual_cam_focal_length = (FocalX + FocalY) * 0.5f;
        digital_zoom_apply = actual_cam_focal_length / (utils.F2 ? Model.MEDMODEL_F2_FL : Model.MEDMODEL_FL);
        OffsetX = CenterX - (frameSize[0] * 0.5f);
        OffsetY = CenterY - (frameSize[1] * 0.5f);

        CameraIntrinsics = new float[] {
                FocalX, 0.0f, frameSize[0] * 0.5f + OffsetX * digital_zoom_apply,
                0.0f, FocalY, frameSize[1] * 0.5f + OffsetY * digital_zoom_apply,
                0.0f,   0.0f, 1.0f
        };

        cam_intrinsics = Nd4j.createFromArray(new float[][]{
                { CameraIntrinsics[0],  0.0f,  CameraIntrinsics[2]},
                {0.0f,  CameraIntrinsics[4],  CameraIntrinsics[5]},
                {0.0f,  0.0f,  1.0f}
        });
    }
}
