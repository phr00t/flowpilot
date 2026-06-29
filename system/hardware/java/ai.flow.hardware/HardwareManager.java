package ai.flow.hardware;

public abstract class HardwareManager {
    public void setBrightness(float brightness){}
    public void turnOffScreen(boolean doTurnOff){}
    public void enableCPUWakeLock(boolean enable){}
    public void enableScreenWakeLock(boolean enable){}
    // Speak a short advisory phrase out loud (text-to-speech). No-op on platforms
    // without TTS. Used by the ELM327 advisory mode to announce the driving actions
    // openpilot would take when it cannot control the car.
    public void announce(String text){}
;}
