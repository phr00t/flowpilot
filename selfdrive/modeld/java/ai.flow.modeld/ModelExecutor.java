package ai.flow.modeld;

import ai.flow.definitions.Definitions;

public abstract class ModelExecutor {
    public static ModelExecutor instance;
    public void init(){}
    public long getIterationRate(){return 0;}
    public float getFrameDropPercent() {return 0f;}
    public boolean isRunning() {return false;}
    public void ExecuteModel(Definitions.FrameData.Reader roadData, Definitions.FrameBuffer.Reader roadBuf,
                             long processStartTimestamp) {}
    public boolean isInitialized() {return false;}
    public void dispose(){}
    public void stop() {}
    public void start() {}
}
