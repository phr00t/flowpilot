package ai.flow.app;

import ai.flow.common.OBDData;
import ai.flow.common.ParamsInterface;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ScreenAdapter;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.scenes.scene2d.InputEvent;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.Label;
import com.badlogic.gdx.scenes.scene2d.ui.Table;
import com.badlogic.gdx.scenes.scene2d.ui.TextButton;
import com.badlogic.gdx.scenes.scene2d.utils.ClickListener;
import com.badlogic.gdx.utils.viewport.FitViewport;

import static ai.flow.app.FlowUI.getPaddedButton;

/**
 * Read-only OBD-II diagnostics screen for a Bluetooth ELM327 adapter.
 *
 * This screen intentionally carries a prominent warning: an ELM327 adapter is a diagnostic
 * tool only and CANNOT be used as an alternative to a comma device / panda for driving.
 * It cannot control steering/gas/brake and openpilot will not engage in this mode.
 */
public class OBDDiagnosticScreen extends ScreenAdapter {

    private final FlowUI appContext;
    private final ParamsInterface params = ParamsInterface.getInstance();
    private final Stage stage;

    private final Label statusValue, speedValue, rpmValue, tempValue, throttleValue, dtcCountValue, dtcValue;

    public OBDDiagnosticScreen(FlowUI appContext) {
        this.appContext = appContext;
        stage = new Stage(new FitViewport(1280, 720));

        Table root = new Table();
        root.setFillParent(true);
        root.pad(40);
        root.top();

        Label title = new Label("OBD-II Diagnostics (ELM327)", appContext.skin, "default-font", "white");
        title.setFontScale(1.4f);
        root.add(title).left().padBottom(10);
        root.row();

        // Unmissable warning about what this mode is NOT.
        Label warning = new Label(
                "WARNING: An ELM327 Bluetooth adapter is a DIAGNOSTIC TOOL ONLY.\n"
              + "It is NOT a replacement for a comma device / panda. It cannot read the\n"
              + "control CAN buses or send steering / gas / brake commands, so openpilot\n"
              + "driving features are DISABLED and the car will NOT be controlled in this mode.",
                appContext.skin, "default-font", "white");
        warning.setColor(Color.SCARLET);
        root.add(warning).left().padBottom(20);
        root.row();

        Table data = new Table();
        statusValue = addRow(data, "Status");
        speedValue = addRow(data, "Vehicle Speed");
        rpmValue = addRow(data, "Engine RPM");
        tempValue = addRow(data, "Coolant Temp");
        throttleValue = addRow(data, "Throttle");
        dtcCountValue = addRow(data, "Stored Fault Codes");
        dtcValue = addRow(data, "Codes");
        root.add(data).left().expandX().fillX();
        root.row();

        if (!params.getBool("UseELM327")) {
            Label hint = new Label(
                    "ELM327 mode is off. Enable \"Use ELM327 OBD (Diagnostic Only)\" in Toggles,\n"
                  + "then pair the adapter in Android Bluetooth settings.",
                    appContext.skin, "default-font", "white");
            hint.setColor(Color.GOLD);
            root.add(hint).left().padTop(20);
            root.row();
        }

        TextButton back = getPaddedButton("BACK", appContext.skin, 5);
        back.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                appContext.setScreen(appContext.settingsScreen);
            }
        });
        root.add(back).left().padTop(30);

        stage.addActor(root);
    }

    private Label addRow(Table table, String key) {
        table.add(new Label(key, appContext.skin, "default-font", "white")).left().pad(8, 0, 8, 60);
        Label value = new Label("--", appContext.skin, "default-font", "white");
        table.add(value).left().pad(8, 0, 8, 0);
        table.row();
        return value;
    }

    private static String fmt(float v, String suffix) {
        return Float.isNaN(v) ? "--" : String.format("%.0f%s", v, suffix);
    }

    private void refresh() {
        statusValue.setText(OBDData.statusMessage);
        statusValue.setColor(OBDData.connected ? Color.LIME : Color.GOLD);
        speedValue.setText(fmt(OBDData.speedKph, " km/h"));
        rpmValue.setText(fmt(OBDData.rpm, " rpm"));
        tempValue.setText(fmt(OBDData.coolantTempC, " C"));
        throttleValue.setText(fmt(OBDData.throttlePct, " %"));
        dtcCountValue.setText(OBDData.dtcCount < 0 ? "--" : Integer.toString(OBDData.dtcCount));
        dtcValue.setText(OBDData.dtcs == null || OBDData.dtcs.isEmpty() ? "none" : OBDData.dtcs);
    }

    @Override
    public void show() {
        Gdx.input.setInputProcessor(stage);
    }

    @Override
    public void render(float delta) {
        Gdx.gl.glClearColor(0f, 0f, 0f, 1f);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);
        refresh();
        stage.act(delta);
        stage.draw();
    }

    @Override
    public void resize(int width, int height) {
        stage.getViewport().update(width, height, true);
    }

    @Override
    public void hide() {
        Gdx.input.setInputProcessor(null);
    }

    @Override
    public void dispose() {
        stage.dispose();
    }
}
