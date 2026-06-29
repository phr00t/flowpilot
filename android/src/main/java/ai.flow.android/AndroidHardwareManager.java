package ai.flow.android;

import ai.flow.hardware.HardwareManager;
import android.speech.tts.TextToSpeech;
import android.view.Window;
import android.view.WindowManager;

import java.util.Locale;

public class AndroidHardwareManager extends HardwareManager {
    public Window window;
    private TextToSpeech tts;
    private volatile boolean ttsReady = false;

    public AndroidHardwareManager(Window window){
        this.window = window;
    }

    public void enableScreenWakeLock(boolean enable){
        if (enable)
            window.addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        else
            window.clearFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
    }

    @Override
    public void announce(String text){
        if (text == null || text.isEmpty())
            return;
        if (tts == null)
            initTts();
        if (ttsReady)
            tts.speak(text, TextToSpeech.QUEUE_FLUSH, null, "advisory");
    }

    private synchronized void initTts(){
        if (tts != null)
            return;
        try {
            tts = new TextToSpeech(window.getContext().getApplicationContext(), status -> {
                if (status == TextToSpeech.SUCCESS) {
                    tts.setLanguage(Locale.US);
                    ttsReady = true;
                }
            });
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
