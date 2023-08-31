package ai.flow.app;

import ai.flow.app.helpers.Utils;
import ai.flow.common.transformations.Camera;
import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.ScreenAdapter;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.scenes.scene2d.Actor;
import com.badlogic.gdx.scenes.scene2d.InputEvent;
import com.badlogic.gdx.scenes.scene2d.Stage;
import com.badlogic.gdx.scenes.scene2d.ui.*;
import com.badlogic.gdx.scenes.scene2d.utils.ChangeListener;
import com.badlogic.gdx.scenes.scene2d.utils.ClickListener;
import com.badlogic.gdx.utils.Align;
import com.badlogic.gdx.utils.viewport.FitViewport;
import ai.flow.app.CalibrationScreens.CalibrationInfo;
import ai.flow.common.ParamsInterface;
import ai.flow.common.SystemUtils;
import ai.flow.common.utils;

import static ai.flow.app.FlowUI.getPaddedButton;

import java.util.ArrayList;


public class SettingsScreen extends ScreenAdapter {

    FlowUI appContext;
    ParamsInterface params = ParamsInterface.getInstance();
    Stage stage;
    TextButton buttonDevice, buttonCalibrate, buttonWideCalibrate, buttonCalibrateExtrinsic,
            buttonTraining, buttonPowerOff, buttonReboot, buttonSoftware,
            buttonUninstall, buttonToggle, buttonCheckUpdate, buttonLogOut;
    ImageButton closeButton;

    SpriteBatch batch;
    Table rootTable, settingTable, scrollTable, currentSettingTable;
    Texture lineTex = Utils.getLineTexture(700, 1, Color.WHITE);
    ScrollPane scrollPane;
    Dialog dialog;
    public java.util.List<String> AdditionalToggles = new java.util.ArrayList<>();
    public java.util.List<TextButton> ToggleButtons = new ArrayList<>();

    public void addKeyValueTable(Table table, String key, String value, boolean addLine) {
        table.add(new Label(key, appContext.skin, "default-font", "white")).left().pad(30);
        table.add(new Label(value, appContext.skin, "default-font", "white")).right().pad(30);
        if (addLine) {
            table.row();
            table.add(new Image(lineTex)).colspan(2).pad(10, 0, 10, 0).height(1);
            table.row();
        }
    }

    public void addKeyValueTable(Table table, String key, Button value, boolean addLine) {
        table.add(new Label(key, appContext.skin, "default-font", "white")).left().pad(30);
        table.add(value).right().pad(30);
        if (addLine) {
            table.row();
            table.add(new Image(lineTex)).colspan(2).pad(10, 0, 10, 0).height(1);
            table.row();
        }
        else
            table.row();
    }

    public void fillDeviceSettings(){
        currentSettingTable.clear();
        String dongleID = params.exists("DongleId") ? params.getString("DongleId") : "N/A";
        addKeyValueTable(currentSettingTable, "Dongle ID", dongleID, true);
        String deviceManufacturer = params.exists("DeviceManufacturer") ? params.getString("DeviceManufacturer") : "";
        addKeyValueTable(currentSettingTable, "Device Manufacturer", deviceManufacturer, true);
        String deviceModel = params.exists("DeviceModel") ? params.getString("DeviceModel") : "";
        addKeyValueTable(currentSettingTable, "Device Name", deviceModel, true);
        addKeyValueTable(currentSettingTable, "Log Out", buttonLogOut, true);
        addKeyValueTable(currentSettingTable, "Reset Extrinsic Calibration", buttonCalibrateExtrinsic, true);
        addKeyValueTable(currentSettingTable, "Review Training Guide", buttonTraining, true);
        currentSettingTable.add(buttonReboot).pad(20);
        currentSettingTable.add(buttonPowerOff).pad(20);
    }

    public void fillSoftwareSettings(){
        currentSettingTable.clear();
        String version = params.exists("Version") ? "flowpilot v" + params.getString("Version") : "";
        addKeyValueTable(currentSettingTable, "Version", version, true);

        addKeyValueTable(currentSettingTable, "Last Updated", "", true);
        addKeyValueTable(currentSettingTable, "Check For Update", buttonCheckUpdate, true);

        String branch = params.exists("GitBranch") ? params.getString("GitBranch") : "";
        addKeyValueTable(currentSettingTable, "Git Branch", branch, true);

        String commit = params.exists("GitCommit") ? params.getString("GitCommit").substring(0, 10) : "";
        addKeyValueTable(currentSettingTable, "Git Commit", commit, true);

        addKeyValueTable(currentSettingTable, "Device Type", SystemUtils.getPlatform(), true);
        addKeyValueTable(currentSettingTable, "Uninstall FlowPilot", buttonUninstall, false);
    }

    public void fillToggleSettings(){
        currentSettingTable.clear();
        for (int i=0; i<ToggleButtons.size(); i++) {
            addKeyValueTable(currentSettingTable, AdditionalToggles.get(i*2), ToggleButtons.get(i), true);
        }
    }

    public SettingsScreen(FlowUI appContext) {
        this.appContext = appContext;

        // easy toggles here. Pairs of string entries, first being display name and second being the parameter that gets set
        AdditionalToggles.add("Stop Sign/Red Lights");
        AdditionalToggles.add("UseAccel");
        AdditionalToggles.add("Use DistSpeed Estimator");
        AdditionalToggles.add("UseDistSpeed");
        AdditionalToggles.add("More Sensitive Slower");
        AdditionalToggles.add("SensitiveSlow");

        stage = new Stage(new FitViewport(1280, 720));
        batch = new SpriteBatch();

        rootTable = new Table();
        rootTable.setFillParent(true);

        settingTable = new Table();
        scrollTable = new Table();
        currentSettingTable = new Table();

        rootTable.add(settingTable);
        rootTable.add(scrollTable).pad(10).padLeft(20);

        scrollPane = new ScrollPane(currentSettingTable);
        scrollPane.setSmoothScrolling(true);
        scrollTable.add(scrollPane);
        scrollTable.setBackground(Utils.createRoundedRectangle(800, 700, 20, new Color(0.18f, 0.18f, 0.18f, 0.8f)));

        closeButton = Utils.getImageButton("selfdrive/assets/icons/icon_close.png");
        closeButton.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                appContext.setScreen(appContext.onRoadScreen);
            }
        });
        closeButton.setColor(1, 1, 1, 0.7f);
        settingTable.add(closeButton).align(Align.left).padLeft(100).padBottom(70).size(70);
        settingTable.row();

        buttonDevice = getPaddedButton("Device", appContext.skin, "no-bg-bold", 5);
        buttonDevice.setChecked(true);
        buttonDevice.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                fillDeviceSettings();
            }
        });
        settingTable.add(buttonDevice).pad(10).align(Align.right);
        settingTable.row();

        buttonSoftware = getPaddedButton("Software", appContext.skin, "no-bg-bold", 5);
        buttonSoftware.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                fillSoftwareSettings();
            }
        });
        settingTable.add(buttonSoftware).pad(10).align(Align.right);
        settingTable.row();

        buttonToggle = getPaddedButton("Toggles", appContext.skin, "no-bg-bold", 5);
        buttonToggle.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                fillToggleSettings();
            }
        });
        settingTable.add(buttonToggle).pad(10).align(Align.right);
        settingTable.row();

        buttonCalibrate = getPaddedButton("RESET", appContext.skin, 5);
        buttonCalibrate.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                appContext.setScreen(new CalibrationInfo(appContext, Camera.CAMERA_TYPE_ROAD,true));
            }
        });

        buttonWideCalibrate = getPaddedButton("RESET", appContext.skin, 5);
        buttonWideCalibrate.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                appContext.setScreen(new CalibrationInfo(appContext, Camera.CAMERA_TYPE_WIDE,true));
            }
        });

        buttonCalibrateExtrinsic = getPaddedButton("RESET", appContext.skin, 5);
        buttonCalibrateExtrinsic.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                dialog = new Dialog("confirm", appContext.skin) {
                    public void result(Object obj) {
                        if (obj.equals(true)) {
                            params.putBool("ResetExtrinsicCalibration", true);
                            dialog.hide();
                        }
                    }
                };
                dialog.text("Are you sure ?");
                dialog.button(getPaddedButton("Yes", appContext.skin, 5), true);
                dialog.button(getPaddedButton("No", appContext.skin, "blue", 5), false);
                dialog.getContentTable().pad(20);
                dialog.show(stage);
            }
        });

        buttonTraining = getPaddedButton("REVIEW", appContext.skin, 5);
        buttonTraining.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                appContext.setScreen(new TrainingScreen(appContext));
            }
        });

        buttonCheckUpdate = getPaddedButton("CHECK", appContext.skin, 5);
        buttonCheckUpdate.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
            }
        });

        buttonLogOut = getPaddedButton("LOG OUT", appContext.skin, 5);
        buttonLogOut.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
                dialog = new Dialog("confirm", appContext.skin) {
                            public void result(Object obj) {
                                if (obj.equals(true)) {
                                    params.deleteKey("UserID");
                                    params.deleteKey("UserToken");
                                    dialog.hide();
                                }
                            }
                        };
                dialog.text("Are you sure ?");
                dialog.button(getPaddedButton("Yes", appContext.skin, 5), true);
                dialog.button(getPaddedButton("No", appContext.skin, "blue", 5), false);
                dialog.getContentTable().pad(20);
                dialog.show(stage);
            }
        });


        buttonReboot = getPaddedButton("Reboot", appContext.skin, 5);
        buttonReboot.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
            }
        });

        buttonPowerOff = getPaddedButton("Power Off", appContext.skin, "critical", 5);
        buttonPowerOff.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
            }
        });

        buttonUninstall = getPaddedButton("UNINSTALL", appContext.skin, "critical", 5);
        buttonUninstall.addListener(new ClickListener() {
            @Override
            public void clicked(InputEvent event, float x, float y) {
            }
        });

        //TODO: Get better toggle buttons.
        for (int i=0; i<AdditionalToggles.size(); i+=2) {
            TextButton nb = new TextButton("  ", appContext.skin, "toggle");
            nb.setChecked(params.exists(AdditionalToggles.get(i + 1)) && params.getBool(AdditionalToggles.get(i + 1)));
            int finalI = i;
            nb.addListener(new ChangeListener() {
                @Override
                public void changed(ChangeListener.ChangeEvent event, Actor actor) {
                    params.putBool(AdditionalToggles.get(finalI + 1), nb.isChecked());
                }
            });
            ToggleButtons.add(nb);
        }

        fillDeviceSettings();

        ButtonGroup buttonGroup = new ButtonGroup(buttonDevice, buttonSoftware, buttonToggle);
        buttonGroup.setMaxCheckCount(1);
        buttonGroup.setUncheckLast(true);

        stage.addActor(rootTable);
    }

    @Override
    public void show() {
        Gdx.input.setInputProcessor(stage);
    }

    @Override
    public void render(float delta) {
        Gdx.gl.glClearColor(.0f, .0f, .0f, 1.0f);
        Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT);

        stage.act(Gdx.graphics.getDeltaTime());
        stage.draw();
    }

    public void resize(int width, int height) {
        stage.getViewport().update(width, height);
    }

    public void hide() {
        Gdx.input.setInputProcessor(null);
    }

    @Override
    public void dispose() {
        Gdx.input.setInputProcessor(null);
        stage.dispose();
    }
}
