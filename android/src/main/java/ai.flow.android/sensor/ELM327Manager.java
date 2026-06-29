package ai.flow.android.sensor;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.Set;
import java.util.UUID;

import ai.flow.common.OBDData;
import ai.flow.sensor.SensorInterface;

/**
 * Read-only OBD-II diagnostics via a Bluetooth ELM327 adapter (e.g. "HH OBD Advanced").
 *
 * IMPORTANT: an ELM327 dongle is a diagnostic tool. It CANNOT replace a panda / comma
 * device for driving: it cannot read the raw control CAN buses at the required rate, and
 * it cannot transmit the steering/gas/brake control frames openpilot needs. This class is
 * therefore strictly read-only and never participates in the control (sendcan) path. When
 * ELM327 mode is enabled, openpilot engagement is hard-disabled elsewhere (see controlsd.py
 * "UseELM327" gate) and the UI shows a warning.
 *
 * The values polled here are surfaced to the UI (OBDDiagnosticScreen) via thread-safe
 * static fields; they are not published onto the control messaging bus.
 */
public class ELM327Manager extends SensorInterface {

    // Standard Serial Port Profile UUID used by ELM327 Bluetooth adapters.
    private static final UUID SPP_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private static final int POLL_INTERVAL_MS = 250;
    private static final int READ_TIMEOUT_MS = 2000;

    // Read-only diagnostics are published to the shared OBDData holder (read by the UI).

    private final Context context;
    private volatile boolean running = false;
    private Thread worker;
    private BluetoothSocket socket;
    private InputStream in;
    private OutputStream out;

    public ELM327Manager(Context context) {
        this.context = context;
    }

    @Override
    public boolean isRunning() {
        return running;
    }

    @Override
    public void start() {
        if (running)
            return;
        running = true;
        OBDData.enabled = true;
        worker = new Thread(this::run, "ELM327Manager");
        worker.setDaemon(true);
        worker.start();
    }

    @Override
    public void stop() {
        running = false;
        closeQuietly();
        OBDData.connected = false;
        OBDData.statusMessage = "ELM327: stopped";
    }

    @Override
    public void dispose() {
        stop();
    }

    private void run() {
        while (running) {
            if (!connect()) {
                // back off before retrying so we don't spin on a missing/unpaired adapter
                sleep(3000);
                continue;
            }
            try {
                initElm();
                while (running && OBDData.connected) {
                    pollOnce();
                    sleep(POLL_INTERVAL_MS);
                }
            } catch (Exception e) {
                OBDData.statusMessage = "ELM327: connection lost (" + e.getMessage() + ")";
            } finally {
                closeQuietly();
                OBDData.connected = false;
            }
        }
    }

    @SuppressLint("MissingPermission")
    private boolean connect() {
        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        if (adapter == null) {
            OBDData.statusMessage = "ELM327: device has no Bluetooth";
            return false;
        }
        if (!adapter.isEnabled()) {
            OBDData.statusMessage = "ELM327: Bluetooth is off";
            return false;
        }

        BluetoothDevice device = findAdapter(adapter);
        if (device == null) {
            OBDData.statusMessage = "ELM327: no paired OBD adapter found (pair it in Android settings)";
            return false;
        }

        try {
            OBDData.adapterName = safeName(device);
            OBDData.statusMessage = "ELM327: connecting to " + OBDData.adapterName + "...";
            socket = device.createRfcommSocketToServiceRecord(SPP_UUID);
            adapter.cancelDiscovery();
            socket.connect();
            in = socket.getInputStream();
            out = socket.getOutputStream();
            OBDData.connected = true;
            OBDData.statusMessage = "ELM327: connected to " + OBDData.adapterName;
            return true;
        } catch (Exception e) {
            OBDData.statusMessage = "ELM327: connect failed (" + e.getMessage() + ")";
            closeQuietly();
            return false;
        }
    }

    @SuppressLint("MissingPermission")
    private BluetoothDevice findAdapter(BluetoothAdapter adapter) {
        try {
            Set<BluetoothDevice> bonded = adapter.getBondedDevices();
            if (bonded == null || bonded.isEmpty())
                return null;
            // Prefer a device whose name looks like an OBD/ELM adapter.
            for (BluetoothDevice d : bonded) {
                String name = safeName(d).toUpperCase();
                if (name.contains("OBD") || name.contains("ELM") || name.contains("VLINK")
                        || name.contains("V-LINK") || name.contains("VIECAR")) {
                    return d;
                }
            }
            // Fall back to the only paired device if there is exactly one.
            if (bonded.size() == 1)
                return bonded.iterator().next();
        } catch (SecurityException e) {
            OBDData.statusMessage = "ELM327: missing Bluetooth permission";
        }
        return null;
    }

    @SuppressLint("MissingPermission")
    private String safeName(BluetoothDevice d) {
        try {
            String n = d.getName();
            return n == null ? d.getAddress() : n;
        } catch (SecurityException e) {
            return "OBD adapter";
        }
    }

    // ELM327 initialisation handshake.
    private void initElm() throws Exception {
        sendCommand("ATZ");    // reset
        sleep(1000);
        sendCommand("ATE0");   // echo off
        sendCommand("ATL0");   // linefeeds off
        sendCommand("ATS0");   // spaces off
        sendCommand("ATH0");   // headers off
        sendCommand("ATSP0");  // auto-detect protocol
    }

    private void pollOnce() throws Exception {
        OBDData.speedKph = parseSpeed(sendCommand("010D"));
        OBDData.rpm = parseRpm(sendCommand("010C"));
        OBDData.coolantTempC = parseTemp(sendCommand("0105"));
        OBDData.throttlePct = parseThrottle(sendCommand("0111"));
        OBDData.dtcCount = parseDtcCount(sendCommand("0101"));
        OBDData.dtcs = parseDtcs(sendCommand("03"));
        OBDData.lastUpdateMs = System.currentTimeMillis();
    }

    /** Write an OBD/AT command and read the response up to the ELM327 '>' prompt. */
    private synchronized String sendCommand(String cmd) throws Exception {
        if (out == null || in == null)
            throw new IllegalStateException("not connected");
        out.write((cmd + "\r").getBytes());
        out.flush();

        StringBuilder sb = new StringBuilder();
        long deadline = System.currentTimeMillis() + READ_TIMEOUT_MS;
        while (System.currentTimeMillis() < deadline) {
            if (in.available() > 0) {
                int c = in.read();
                if (c == -1)
                    break;
                if (c == '>')   // prompt -> end of response
                    break;
                if (c != '\r' && c != '\n')
                    sb.append((char) c);
            } else {
                sleep(5);
            }
        }
        return sb.toString().trim();
    }

    // ---- OBD-II PID response parsing ----
    // Responses look like "410D3C" (speed), "410C1AF8" (rpm), etc. with the leading
    // "41" mode byte and the PID echoed back. Returns NaN when the value is unavailable.

    private static int[] dataBytes(String resp, String expectedPrefix) {
        if (resp == null)
            return null;
        String hex = resp.replaceAll("[^0-9A-Fa-f]", "").toUpperCase();
        int idx = hex.indexOf(expectedPrefix);
        if (idx < 0 || hex.length() < idx + expectedPrefix.length() + 2)
            return null;
        String payload = hex.substring(idx + expectedPrefix.length());
        if (payload.length() % 2 != 0)
            payload = payload.substring(0, payload.length() - 1);
        int n = payload.length() / 2;
        int[] bytes = new int[n];
        try {
            for (int i = 0; i < n; i++)
                bytes[i] = Integer.parseInt(payload.substring(i * 2, i * 2 + 2), 16);
        } catch (NumberFormatException e) {
            return null;
        }
        return bytes;
    }

    private static float parseSpeed(String resp) {
        int[] b = dataBytes(resp, "410D");
        return (b != null && b.length >= 1) ? b[0] : Float.NaN;     // km/h
    }

    private static float parseRpm(String resp) {
        int[] b = dataBytes(resp, "410C");
        return (b != null && b.length >= 2) ? (256 * b[0] + b[1]) / 4f : Float.NaN;
    }

    private static float parseTemp(String resp) {
        int[] b = dataBytes(resp, "4105");
        return (b != null && b.length >= 1) ? b[0] - 40 : Float.NaN; // deg C
    }

    private static float parseThrottle(String resp) {
        int[] b = dataBytes(resp, "4111");
        return (b != null && b.length >= 1) ? b[0] * 100f / 255f : Float.NaN; // %
    }

    private static int parseDtcCount(String resp) {
        int[] b = dataBytes(resp, "4101");
        // bit7 of the first byte is MIL on; bits0-6 are the stored DTC count.
        return (b != null && b.length >= 1) ? (b[0] & 0x7F) : -1;
    }

    /** Decode mode 03 stored DTCs into J2012 strings (e.g. P0133). */
    private static String parseDtcs(String resp) {
        if (resp == null)
            return "";
        String hex = resp.replaceAll("[^0-9A-Fa-f]", "").toUpperCase();
        int idx = hex.indexOf("43");
        if (idx < 0)
            return "";
        String payload = hex.substring(idx + 2);
        StringBuilder out = new StringBuilder();
        for (int i = 0; i + 4 <= payload.length(); i += 4) {
            String code = decodeDtc(payload.substring(i, i + 4));
            if (code != null) {
                if (out.length() > 0)
                    out.append(", ");
                out.append(code);
            }
        }
        return out.toString();
    }

    private static String decodeDtc(String fourHex) {
        try {
            int a = Integer.parseInt(fourHex.substring(0, 2), 16);
            int b = Integer.parseInt(fourHex.substring(2, 4), 16);
            if (a == 0 && b == 0)
                return null; // padding / no code
            char[] system = {'P', 'C', 'B', 'U'};
            char first = system[(a & 0xC0) >> 6];
            int d1 = (a & 0x30) >> 4;
            int d2 = a & 0x0F;
            int d3 = (b & 0xF0) >> 4;
            int d4 = b & 0x0F;
            return String.format("%c%d%X%X%X", first, d1, d2, d3, d4);
        } catch (Exception e) {
            return null;
        }
    }

    private void closeQuietly() {
        try { if (in != null) in.close(); } catch (Exception ignored) {}
        try { if (out != null) out.close(); } catch (Exception ignored) {}
        try { if (socket != null) socket.close(); } catch (Exception ignored) {}
        in = null;
        out = null;
        socket = null;
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
