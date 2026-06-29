package ai.flow.common;

/**
 * Shared, read-only snapshot of OBD-II diagnostics from a Bluetooth ELM327 adapter.
 *
 * The android module's ELM327Manager writes these fields; the UI module's
 * OBDDiagnosticScreen reads them. This holder lives in the common module so both
 * sides can reference it without a circular module dependency.
 *
 * These values are purely informational. An ELM327 cannot control the car, so nothing
 * here ever feeds the openpilot control path.
 */
public class OBDData {
    public static volatile boolean enabled = false;       // user selected ELM327 mode
    public static volatile boolean connected = false;     // adapter link is up
    public static volatile String statusMessage = "ELM327 mode is off";
    public static volatile String adapterName = "";
    public static volatile float speedKph = Float.NaN;
    public static volatile float rpm = Float.NaN;
    public static volatile float coolantTempC = Float.NaN;
    public static volatile float throttlePct = Float.NaN;
    public static volatile int dtcCount = -1;
    public static volatile String dtcs = "";
    public static volatile long lastUpdateMs = 0;
}
