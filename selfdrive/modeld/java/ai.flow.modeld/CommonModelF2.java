package ai.flow.modeld;

// Core java classes

public class CommonModelF2 {

    /* NUCLEAR GRADE MODEL OUTPUT
    1. `ModelOutputLeads`: `2 * sizeof(ModelOutputLeadPrediction) + sizeof(float) * LEAD_MHP_SELECTION` = `2 * 204 + 3 * 4 = 420 bytes`.

    So, the corrected size of the `ModelOutput` structure is:

    `sizeof(ModelPlans) + sizeof(ModelLaneLines) + sizeof(ModelRoadEdges) + sizeof(ModelLeads) + sizeof(ModelStopLines) + sizeof(ModelMeta) + sizeof(ModelPose)`

    = `19820 bytes (for ModelOutputPlans)`
    + `2144 bytes (for ModelOutputLaneLines)`
    + `1056 bytes (for ModelOutputRoadEdges)`
    + `420 bytes (for ModelOutputLeads, corrected)`
    + `208 bytes (for ModelOutputStopLines)`
    + `400 bytes (for ModelOutputMeta)`
    + `48 bytes (for ModelOutputPose)`

    = **26096 bytes**

    This matches the size you mentioned, so it seems like the calculation is correct this time! I apologize for the earlier confusion and thank you for your patience.

1. `ModelOutputXYZ`: 3 * 4 = 12 bytes
2. `ModelOutputYZ`: 2 * 4 = 8 bytes
3. `ModelOutputPlanElement`: 5 * 12 = 60 bytes
4. `ModelOutputPlanPrediction`: (2 * 33 * 60) + 4 = 3964 bytes
5. `ModelOutputPlans`: 5 * 3964 = 19820 bytes
6. `ModelOutputLinesXY`: 4 * 33 * 8 = 1056 bytes
7. `ModelOutputLineProbVal`: 2 * 4 = 8 bytes
8. `ModelOutputLinesProb`: 4 * 8 = 32 bytes
9. `ModelOutputLaneLines`: (2 * 1056) + 32 = 2144 bytes
10. `ModelOutputEdgessXY`: 2 * 33 * 8 = 528 bytes
11. `ModelOutputRoadEdges`: (2 * 528) = 1056 bytes
12. `ModelOutputLeadElement`: 4 * 4 = 16 bytes
13. `ModelOutputLeadPrediction`: (2 * LEAD_TRAJ_LEN * sizeof(ModelOutputLeadElement)) + (sizeof(float)*LEAD_MHP_SELECTION) = (2 * LEAD_TRAJ_LEN *16) + (3*4) = (2*6*16)+(3*4)=204 bytes.
14. `ModelOutputLeads`: LEAD_MHP_N*sizeof(ModelOutputLeadPrediction) + sizeof(float)*LEAD_MHP_SELECTION = LEAD_MHP_N*204 + LEAD_MHP_SELECTION*4=2*204+3*4=420 bytes.
15. `ModelOutputStopLineElement` : (sizeof(ModelOutputXYZ)*2 + sizeof(float)*2) = (12*2+4*2)=32 bytes.
16. `ModelOutputStopLinePrediction` : sizeof(ModelOutputStopLineElement)*2 + sizeof(float) =32*2+4=68 bytes.
17. `ModelOutputStopLines` : STOP_LINE_MHP_N*sizeof(ModelOutputStopLinePrediction) + sizeof(float)=3*68+4=208 bytes.
18. `ModelOutputPose` : sizeof(ModelOutputXYZ)*4=12*4=48 bytes.
19. `ModelOutputDisengageProb` : sizeof(float)*DISENGAGE_LEN=7*4=28 bytes.
20. `ModelOutputBlinkerProb` : sizeof(float)*BLINKER_LEN=6*4=24 bytes.
21. `ModelOutputDesireProb` : sizeof(float)*DESIRE_LEN=8*4=32 bytes.
22. `ModelOutputMeta` : sizeof(ModelOutputDesireProb)+sizeof(float)+sizeof(ModelOutputDisengageProb)*DISENGAGE_LEN+sizeof(ModelOutputBlinkerProb)*BLINKER_LEN+sizeof(ModelOutputDesireProb)*DESIRE_PRED_LEN=32+4+28*5+24*6+32*4=400 bytes.
     */

    public static final int DESIRE_LEN = 8;
    public static final int TRAJECTORY_SIZE = 33;

    public static final int DESIRE_PRED_LEN = 4;
    public static final int OTHER_META_SIZE = 32;
    public static final int NUM_META_INTERVALS = 5;
    public static final int META_STRIDE = 7;

    public static final int PLAN_MHP_N = 5;
    public static final int PLAN_MHP_COLUMNS = 15;
    public static final int PLAN_MHP_VALS = 15*33;
    public static final int PLAN_MHP_SELECTION = 1;
    public static final int PLAN_MHP_GROUP_SIZE =  (2*PLAN_MHP_VALS + PLAN_MHP_SELECTION);

    public static final int LEAD_MHP_N = 2;
    public static final int LEAD_TRAJ_LEN = 6;
    public static final int LEAD_PRED_DIM = 4;
    public static final int LEAD_MHP_VALS = LEAD_PRED_DIM * LEAD_TRAJ_LEN;
    public static final int LEAD_MHP_SELECTION = 3;
    public static final int LEAD_MHP_GROUP_SIZE = (2*LEAD_MHP_VALS + LEAD_MHP_SELECTION);

    public static final int POSE_SIZE = 12;

    public static final int PLAN_IDX = 0;
    public static final int LL_IDX = PLAN_IDX + PLAN_MHP_N * PLAN_MHP_GROUP_SIZE;
    public static final int LL_PROB_IDX = LL_IDX + 4*2*2*33;
    public static final int RE_IDX = LL_PROB_IDX + 8;
    public static final int LEAD_IDX = RE_IDX + 2*2*2*33;
    public static final int LEAD_PROB_IDX = LEAD_IDX + LEAD_MHP_N * LEAD_MHP_GROUP_SIZE;
    public static final int DESIRE_STATE_IDX = LEAD_PROB_IDX + 3;
    public static final int META_IDX = DESIRE_STATE_IDX + DESIRE_LEN;
    public static final int POSE_IDX = META_IDX + OTHER_META_SIZE + DESIRE_PRED_LEN;
    public static final int OUTPUT_SIZE =  POSE_IDX + POSE_SIZE;
    public static final int TEMPORAL_SIZE = 512;

    public static final float FCW_THRESHOLD_5MS2_HIGH = 0.15f;
    public static final float FCW_THRESHOLD_5MS2_LOW = 0.05f;
    public static final float FCW_THRESHOLD_3MS2 = 0.7f;

    public static final float[] prev_brake_5ms2_probs = {0f, 0f, 0f, 0f, 0f};
    public static final float[] prev_brake_3ms2_probs = {0f, 0f, 0f};

    public static final  float[] T_IDXS = {0.f, 0.00976562f, 0.0390625f, 0.08789062f, 0.15625f, 0.24414062f,  0.3515625f,  0.47851562f,
            0.625f, 0.79101562f, 0.9765625f, 1.18164062f,  1.40625f,  1.65039062f,  1.9140625f,
            2.19726562f, 2.5f, 2.82226562f, 3.1640625f, 3.52539062f, 3.90625f, 4.30664062f, 4.7265625f, 5.16601562f,
            5.625f, 6.10351562f, 6.6015625f, 7.11914062f, 7.65625f, 8.21289062f, 8.7890625f, 9.38476562f, 10.f};

    public static final float[] X_IDXS = {0.f, 0.1875f, 0.75f, 1.6875f, 3.f, 4.6875f, 6.75f, 9.1875f, 12.f,  15.1875f, 18.75f, 22.6875f,
            27.f,  31.6875f,  36.75f, 42.1875f, 48.f, 54.1875f, 60.75f,  67.6875f,  75.f, 82.6875f, 90.75f, 99.1875f, 108.f, 117.1875f,
            126.75f, 136.6875f, 147.f, 157.6875f, 168.75f, 180.1875f, 192.0f};
    public static final float[] t_offsets = {0.0f, 2.0f, 4.0f};
}

