using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

using Java = import "./include/java.capnp";
$Java.package("ai.flow.definitions");
$Java.outerClassname("Definitions");

using Car = import "car.capnp";

@0xcb6d168e14a64097;

const logVersion :Int32 = 1;

struct Map(Key, Value) {
  entries @0 :List(Entry);
  struct Entry {
    key @0 :Key;
    value @1 :Value;
  }
}

struct InitData {
  kernelArgs @0 :List(Text);
  kernelVersion @15 :Text;
  osVersion @18 :Text;

  dongleId @2 :Text;

  deviceType @3 :DeviceType;
  version @4 :Text;
  gitCommit @10 :Text;
  gitBranch @11 :Text;
  gitRemote @13 :Text;

  androidProperties @16 :Map(Text, Text);

  pandaInfo @8 :PandaInfo;

  dirty @9 :Bool;
  passive @12 :Bool;
  params @17 :Map(Text, Data);

  commands @19 :Map(Text, Data);

  enum DeviceType {
    unknown @0;
    neo @1;
    chffrAndroid @2;
    chffrIos @3;
    tici @4;
    pc @5;
  }

  struct PandaInfo {
    hasPanda @0 :Bool;
    dongleId @1 :Text;
    stVersion @2 :Text;
    espVersion @3 :Text;
  }

  # ***** deprecated stuff *****
  gctxDEPRECATED @1 :Text;
  androidBuildInfo @5 :AndroidBuildInfo;
  androidSensorsDEPRECATED @6 :List(AndroidSensor);
  chffrAndroidExtraDEPRECATED @7 :ChffrAndroidExtra;
  iosBuildInfoDEPRECATED @14 :IosBuildInfo;

  struct AndroidBuildInfo {
    board @0 :Text;
    bootloader @1 :Text;
    brand @2 :Text;
    device @3 :Text;
    display @4 :Text;
    fingerprint @5 :Text;
    hardware @6 :Text;
    host @7 :Text;
    id @8 :Text;
    manufacturer @9 :Text;
    model @10 :Text;
    product @11 :Text;
    radioVersion @12 :Text;
    serial @13 :Text;
    supportedAbis @14 :List(Text);
    tags @15 :Text;
    time @16 :Int64;
    type @17 :Text;
    user @18 :Text;

    versionCodename @19 :Text;
    versionRelease @20 :Text;
    versionSdk @21 :Int32;
    versionSecurityPatch @22 :Text;
  }

  struct AndroidSensor {
    id @0 :Int32;
    name @1 :Text;
    vendor @2 :Text;
    version @3 :Int32;
    handle @4 :Int32;
    type @5 :Int32;
    maxRange @6 :Float32;
    resolution @7 :Float32;
    power @8 :Float32;
    minDelay @9 :Int32;
    fifoReservedEventCount @10 :UInt32;
    fifoMaxEventCount @11 :UInt32;
    stringType @12 :Text;
    maxDelay @13 :Int32;
  }

  struct ChffrAndroidExtra {
    allCameraCharacteristics @0 :Map(Text, Text);
  }

  struct IosBuildInfo {
    appVersion @0 :Text;
    appBuild @1 :UInt32;
    osVersion @2 :Text;
    deviceModel @3 :Text;
  }
}

struct CameraOdometry {
  frameId @4 :UInt32;
  timestampEof @5 :UInt64;
  trans @0 :List(Float32); # m/s in device frame
  rot @1 :List(Float32); # rad/s in device frame
  transStd @2 :List(Float32); # std m/s in device frame
  rotStd @3 :List(Float32); # std rad/s in device frame
  wideFromDeviceEuler @6 :List(Float32);
  wideFromDeviceEulerStd @7 :List(Float32);
  roadTransformTrans @8 :List(Float32);
  roadTransformTransStd @9 :List(Float32);
}

struct Sentinel {
  enum SentinelType {
    endOfSegment @0;
    endOfRoute @1;
    startOfSegment @2;
    startOfRoute @3;
  }
  type @0 :SentinelType;
  signal @1 :Int32;
}

struct LiveCalibrationData {
  calStatus @11 :Status;
  calCycle @2 :Int32;
  calPerc @3 :Int8;
  validBlocks @9 :Int32;

  # view_frame_from_road_frame
  # ui's is inversed needs new
  extrinsicMatrix @4 :List(Float32);
  # the direction of travel vector in device frame
  rpyCalib @7 :List(Float32);
  rpyCalibSpread @8 :List(Float32);
  wideFromDeviceEuler @10 :List(Float32);

  warpMatrixDEPRECATED @0 :List(Float32);
  calStatusDEPRECATED @1 :Int8;
  warpMatrix2DEPRECATED @5 :List(Float32);
  warpMatrixBigDEPRECATED @6 :List(Float32);
  
  enum Status {
    uncalibrated @0;
    calibrated @1;
    invalid @2;
    recalibrating @3;
  }
}

struct ModelDataV2 {
  frameId @0 :UInt32;
  frameIdExtra @20 :UInt32;
  frameAge @1 :UInt32;
  frameDropPerc @2 :Float32;
  timestampEof @3 :UInt64;
  modelExecutionTime @15 :Float32;
  gpuExecutionTime @17 :Float32;
  rawPredictions @16 :Data;

  # predicted future position, orientation, etc..
  position @4 :XYZTData;
  orientation @5 :XYZTData;
  velocity @6 :XYZTData;
  orientationRate @7 :XYZTData;
  acceleration @19 :XYZTData;

  # prediction lanelines and road edges
  laneLines @8 :List(XYZTData);
  laneLineProbs @9 :List(Float32);
  laneLineStds @13 :List(Float32);
  roadEdges @10 :List(XYZTData);
  roadEdgeStds @14 :List(Float32);

  # predicted lead cars
  leads @11 :List(LeadDataV2);
  leadsV3 @18 :List(LeadDataV3);

  meta @12 :MetaData;
  confidence @23: ConfidenceClass;

  # Model perceived motion
  temporalPose @21 :Pose;

  navEnabled @22 :Bool;
  locationMonoTime @24 :UInt64;

  # All SI units and in device frame
  struct XYZTData {
    x @0 :List(Float32);
    y @1 :List(Float32);
    z @2 :List(Float32);
    t @3 :List(Float32);
    xStd @4 :List(Float32);
    yStd @5 :List(Float32);
    zStd @6 :List(Float32);
  }

  struct LeadDataV2 {
    prob @0 :Float32; # probability that car is your lead at time t
    t @1 :Float32;

    # x and y are relative position in device frame
    # v is norm relative speed
    # a is norm relative acceleration
    xyva @2 :List(Float32);
    xyvaStd @3 :List(Float32);
  }

  struct LeadDataV3 {
    prob @0 :Float32; # probability that car is your lead at time t
    probTime @1 :Float32;
    t @2 :List(Float32);

    # x and y are relative position in device frame
    # v absolute norm speed
    # a is derivative of v
    x @3 :List(Float32);
    xStd @4 :List(Float32);
    y @5 :List(Float32);
    yStd @6 :List(Float32);
    v @7 :List(Float32);
    vStd @8 :List(Float32);
    a @9 :List(Float32);
    aStd @10 :List(Float32);
  }


  struct MetaData {
    engagedProb @0 :Float32;
    desirePrediction @1 :List(Float32);
    desireState @5 :List(Float32);
    disengagePredictions @6 :DisengagePredictions;
    hardBrakePredicted @7 :Bool;

    # deprecated
    brakeDisengageProbDEPRECATED @2 :Float32;
    gasDisengageProbDEPRECATED @3 :Float32;
    steerOverrideProbDEPRECATED @4 :Float32;
  }

  enum ConfidenceClass {
    red @0;
    yellow @1;
    green @2;
  }

  struct DisengagePredictions {
    t @0 :List(Float32);
    brakeDisengageProbs @1 :List(Float32);
    gasDisengageProbs @2 :List(Float32);
    steerOverrideProbs @3 :List(Float32);
    brake3MetersPerSecondSquaredProbs @4 :List(Float32);
    brake4MetersPerSecondSquaredProbs @5 :List(Float32);
    brake5MetersPerSecondSquaredProbs @6 :List(Float32);
  }

  struct Pose {
    trans @0 :List(Float32); # m/s in device frame
    rot @1 :List(Float32); # rad/s in device frame
    transStd @2 :List(Float32); # std m/s in device frame
    rotStd @3 :List(Float32); # std rad/s in device frame
  }
}

struct NavModelData {
  frameId @0 :UInt32;
  modelExecutionTime @1 :Float32;
  dspExecutionTime @2 :Float32;
  features @3 :List(Float32);
  # predicted future position
  position @4 :XYData;
  desirePrediction @5 :List(Float32);

  # All SI units and in device frame
  struct XYData {
    x @0 :List(Float32);
    y @1 :List(Float32);
    xStd @2 :List(Float32);
    yStd @3 :List(Float32);
  }
}

struct Desire {
    meta @0: List(Float32);
}

struct Gyroscope {
  x @0: Float32;
  y @1: Float32;
  z @2: Float32;
}

struct Accelerometer {
  x @0: Float32;
  y @1: Float32;
  z @2: Float32;
}

struct FrameData {
  frameId @0 :UInt32;
  encodeId @1 :UInt32; # DEPRECATED

  frameType @7 :FrameType;
  frameLength @3 :Int32;

  # Timestamps
  timestampEof @2 :UInt64;
  timestampSof @8 :UInt64;

  # Exposure
  integLines @4 :Int32;
  highConversionGain @20 :Bool;
  gain @15 :Float32; # This includes highConversionGain if enabled
  measuredGreyFraction @21 :Float32;
  targetGreyFraction @22 :Float32;
  nativeImageAddr @23 :UInt64;

  # Focus
  lensPos @11 :Int32;
  lensSag @12 :Float32;
  lensErr @13 :Float32;
  lensTruePos @14 :Float32;
  focusVal @16 :List(Int16);
  focusConf @17 :List(UInt8);
  sharpnessScore @18 :List(UInt16);
  recoverState @19 :Int32;

  intrinsics @10 :List(Float32);

  androidCaptureResult @9 :AndroidCaptureResult;

  image @6 :Data;
  globalGainDEPRECATED @5 :Int32;

  enum FrameType {
    unknown @0;
    neo @1;
    chffrAndroid @2;
    front @3;
  }

  struct AndroidCaptureResult {
    sensitivity @0 :Int32;
    frameDuration @1 :Int64;
    exposureTime @2 :Int64;
    rollingShutterSkew @3 :UInt64;
    colorCorrectionTransform @4 :List(Int32);
    colorCorrectionGains @5 :List(Float32);
    displayRotation @6 :Int8;
  }
}

struct FrameBuffer {
  image @0 :Data;
  # address is useful to share buffer within same process.
  imageAddress @1 :UInt64;
  encoding @2 :Encoding;

  frameWidth @3 :Int32;
  frameHeight @4 :Int32;

  stride @5 :Int32;
  yWidth @6 :Int32;
  yHeight @7 :Int32;
  yPixelStride @8 :Int32;
  uvWidth @9 :Int32;
  uvHeight @10 :Int32;
  uvPixelStride @11 :Int32;
  uOffset @12 :Int32;
  vOffset @13 :Int32;

  enum Encoding {
    yuv @0;
    rgb @1;
  }
}

struct Memory {
    timestamp @0 :UInt64;

    total @1 :UInt64;
    free @2 :UInt64;
    available @3 :UInt64;
    buffers @4 :UInt64;
    cached @5 :UInt64;
    active @6 :UInt64;
    inactive @7 :UInt64;
    shared @8 :UInt64;
}

struct CanData {
  address @0 :UInt32;
  busTime @1 :UInt16;
  dat     @2 :Data;
  src     @3 :UInt8;
}

struct LateralPlan {
  modelMonoTime @0 :UInt64;
  laneWidth @1 :Float32;
  lProb @2 :Float32;
  rProb @3 :Float32;
  dPathPoints @4 :List(Float32);
  dProb @5 :Float32;

  mpcSolutionValid @6 :Bool;
  desire @7 :Desire;
  laneChangeState @8 :LaneChangeState;
  laneChangeDirection @9 :LaneChangeDirection;
  useLaneLines @10 :Bool;

  # desired curvatures over next 2.5s in rad/m
  psis @11 :List(Float32);
  curvatures @12 :List(Float32);
  curvatureRates @13 :List(Float32);

  solverExecutionTime @14 :Float32;

  enum Desire {
    none @0;
    turnLeft @1;
    turnRight @2;
    laneChangeLeft @3;
    laneChangeRight @4;
    keepLeft @5;
    keepRight @6;
  }

  enum LaneChangeState {
    off @0;
    preLaneChange @1;
    laneChangeStarting @2;
    laneChangeFinishing @3;
  }

  enum LaneChangeDirection {
    none @0;
    left @1;
    right @2;
  }
}

struct ControlsState @0x97ff69c53601abf1 {
  startMonoTime @48 :UInt64;
  longitudinalPlanMonoTime @28 :UInt64;
  lateralPlanMonoTime @50 :UInt64;

  state @31 :FlowpilotState;
  enabled @19 :Bool;
  active @36 :Bool;

  experimentalMode @64 :Bool;

  longControlState @30 :Car.CarControl.Actuators.LongControlState;
  vPid @2 :Float32;
  vTargetLead @3 :Float32;
  vCruise @22 :Float32;  # actual set speed
  vCruiseCluster @63 :Float32;  # set speed to display in the UI
  upAccelCmd @4 :Float32;
  uiAccelCmd @5 :Float32;
  ufAccelCmd @33 :Float32;
  aTarget @35 :Float32;
  curvature @37 :Float32;  # path curvature from vehicle model
  desiredCurvature @61 :Float32;  # lag adjusted curvatures used by lateral controllers
  desiredCurvatureRate @62 :Float32;
  forceDecel @51 :Bool;

  # UI alerts
  alertText1 @24 :Text;
  alertText2 @25 :Text;
  alertStatus @38 :AlertStatus;
  alertSize @39 :AlertSize;
  alertBlinkingRate @42 :Float32;
  alertType @44 :Text;
  alertSound @56 :Car.CarControl.HUDControl.AudibleAlert;
  engageable @41 :Bool;  # can OP be engaged?

  cumLagMs @15 :Float32;
  canErrorCounter @57 :UInt32;

  lateralControlState :union {
    indiState @52 :LateralINDIState;
    pidState @53 :LateralPIDState;
    angleState @58 :LateralAngleState;
    debugState @59 :LateralDebugState;
    torqueState @60 :LateralTorqueState;
    curvatureState @65 :LateralCurvatureState;

    lqrStateDEPRECATED @55 :LateralLQRState;
  }

  enum FlowpilotState @0xdbe58b96d2d1ac61 {
    disabled @0;
    preEnabled @1;
    enabled @2;
    softDisabling @3;
    overriding @4;  # superset of overriding with steering or accelerator
  }

  enum AlertStatus {
    normal @0;       # low priority alert for user's convenience
    userPrompt @1;   # mid priority alert that might require user intervention
    critical @2;     # high priority alert that needs immediate user intervention
  }

  enum AlertSize {
    none @0;    # don't display the alert
    small @1;   # small box
    mid @2;     # mid screen
    full @3;    # full screen
  }

  struct LateralINDIState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    steeringRateDeg @2 :Float32;
    steeringAccelDeg @3 :Float32;
    rateSetPoint @4 :Float32;
    accelSetPoint @5 :Float32;
    accelError @6 :Float32;
    delayedOutput @7 :Float32;
    delta @8 :Float32;
    output @9 :Float32;
    saturated @10 :Bool;
    steeringAngleDesiredDeg @11 :Float32;
    steeringRateDesiredDeg @12 :Float32;
  }

  struct LateralPIDState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    steeringRateDeg @2 :Float32;
    angleError @3 :Float32;
    p @4 :Float32;
    i @5 :Float32;
    f @6 :Float32;
    output @7 :Float32;
    saturated @8 :Bool;
    steeringAngleDesiredDeg @9 :Float32;
   }

  struct LateralTorqueState {
    active @0 :Bool;
    error @1 :Float32;
    errorRate @8 :Float32;
    p @2 :Float32;
    i @3 :Float32;
    d @4 :Float32;
    f @5 :Float32;
    output @6 :Float32;
    saturated @7 :Bool;
    actualLateralAccel @9 :Float32;
    desiredLateralAccel @10 :Float32;
   }

  struct LateralLQRState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    i @2 :Float32;
    output @3 :Float32;
    lqrOutput @4 :Float32;
    saturated @5 :Bool;
    steeringAngleDesiredDeg @6 :Float32;
  }

  struct LateralAngleState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    output @2 :Float32;
    saturated @3 :Bool;
    steeringAngleDesiredDeg @4 :Float32;
  }

  struct LateralCurvatureState {
    active @0 :Bool;
    actualCurvature @1 :Float32;
    desiredCurvature @2 :Float32;
    error @3 :Float32;
    p @4 :Float32;
    i @5 :Float32;
    f @6 :Float32;
    output @7 :Float32;
    saturated @8 :Bool;
  }

  struct LateralDebugState {
    active @0 :Bool;
    steeringAngleDeg @1 :Float32;
    output @2 :Float32;
    saturated @3 :Bool;
  }

  # deprecated
  vEgoDEPRECATED @0 :Float32;
  vEgoRawDEPRECATED @32 :Float32;
  aEgoDEPRECATED @1 :Float32;
  canMonoTimeDEPRECATED @16 :UInt64;
  radarStateMonoTimeDEPRECATED @17 :UInt64;
  mdMonoTimeDEPRECATED @18 :UInt64;
  yActualDEPRECATED @6 :Float32;
  yDesDEPRECATED @7 :Float32;
  upSteerDEPRECATED @8 :Float32;
  uiSteerDEPRECATED @9 :Float32;
  ufSteerDEPRECATED @34 :Float32;
  aTargetMinDEPRECATED @10 :Float32;
  aTargetMaxDEPRECATED @11 :Float32;
  rearViewCamDEPRECATED @23 :Bool;
  driverMonitoringOnDEPRECATED @43 :Bool;
  hudLeadDEPRECATED @14 :Int32;
  alertSoundDEPRECATED @45 :Text;
  angleModelBiasDEPRECATED @27 :Float32;
  gpsPlannerActiveDEPRECATED @40 :Bool;
  decelForTurnDEPRECATED @47 :Bool;
  decelForModelDEPRECATED @54 :Bool;
  awarenessStatusDEPRECATED @26 :Float32;
  angleSteersDEPRECATED @13 :Float32;
  vCurvatureDEPRECATED @46 :Float32;
  mapValidDEPRECATED @49 :Bool;
  jerkFactorDEPRECATED @12 :Float32;
  steerOverrideDEPRECATED @20 :Bool;
  steeringAngleDesiredDegDEPRECATED @29 :Float32;
  canMonoTimesDEPRECATED @21 :List(UInt64);
}

struct RadarState {
  canMonoTimes @0 :List(UInt64);
  mdMonoTime @1 :UInt64;
  carStateMonoTime @2 :UInt64;
  radarErrors @3 :List(Car.RadarData.Error);

  leadOne @4 :LeadData;
  leadTwo @5 :LeadData;
  cumLagMs @6 :Float32;

  struct LeadData {
    dRel @0 :Float32;
    yRel @1 :Float32;
    vRel @2 :Float32;
    aRel @3 :Float32;
    vLead @4 :Float32;
    dPath @5 :Float32;
    vLat @6 :Float32;
    vLeadK @7 :Float32;
    aLeadK @8 :Float32;
    fcw @9 :Bool;
    status @10 :Bool;
    aLeadTau @11 :Float32;
    modelProb @12 :Float32;
    radar @13 :Bool;
  }
}

struct LongitudinalPlan {
  modelMonoTime @0 :UInt64;
  hasLead @1 :Bool;
  fcw @2 :Bool;
  longitudinalPlanSource @3 :LongitudinalPlanSource;
  processingDelay @4 :Float32;

  # desired speed/accel/jerk over next 2.5s
  accels @5 :List(Float32);
  speeds @6 :List(Float32);
  jerks @7 :List(Float32);

  solverExecutionTime @8 :Float32;

  enum LongitudinalPlanSource {
    cruise @0;
    lead0 @1;
    lead1 @2;
    lead2 @3;
    e2e @4;
  }

  struct GpsTrajectory {
    x @0 :List(Float32);
    y @1 :List(Float32);
  }
}

struct SensorEventData {
  version @0 :Int32;
  sensor @1 :Int32;
  type @2 :Int32;
  timestamp @3 :Int64;
  uncalibratedDEPRECATED @10 :Bool;

  union {
    acceleration @4 :SensorVec;
    magnetic @5 :SensorVec;
    orientation @6 :SensorVec;
    gyro @7 :SensorVec;
    pressure @9 :SensorVec;
    magneticUncalibrated @11 :SensorVec;
    gyroUncalibrated @12 :SensorVec;
    proximity @13: Float32;
    light @14: Float32;
    temperature @15: Float32;
  }
  source @8 :SensorSource;

  struct SensorVec {
    v @0 :List(Float32);
    status @1 :Int8;
  }

  enum SensorSource {
    android @0;
    iOS @1;
    fiber @2;
    velodyne @3;  # Velodyne IMU
    bno055 @4;    # Bosch accelerometer
    lsm6ds3 @5;   # accelerometer (c2)
    bmp280 @6;    # barometer (c2)
    mmc3416x @7;  # magnetometer (c2)
    bmx055 @8;
    rpr0521 @9;
    lsm6ds3trc @10;
    mmc5603nj @11;
  }
}

# android struct GpsLocation
struct GpsLocationData {
  # Contains GpsLocationFlags bits.
  flags @0 :UInt16;

  # Represents latitude in degrees.
  latitude @1 :Float64;

  # Represents longitude in degrees.
  longitude @2 :Float64;

  # Represents altitude in meters above the WGS 84 reference ellipsoid.
  altitude @3 :Float64;

  # Represents speed in meters per second.
  speed @4 :Float32;

  # Represents heading in degrees.
  bearingDeg @5 :Float32;

  # Represents expected accuracy in meters. (presumably 1 sigma?)
  accuracy @6 :Float32;

  unixTimestampMillis @7 :Int64;

  source @8 :SensorSource;

  # Represents NED velocity in m/s.
  vNED @9 :List(Float32);

  # Represents expected vertical accuracy in meters. (presumably 1 sigma?)
  verticalAccuracy @10 :Float32;

  # Represents bearing accuracy in degrees. (presumably 1 sigma?)
  bearingAccuracyDeg @11 :Float32;

  # Represents velocity accuracy in m/s. (presumably 1 sigma?)
  speedAccuracy @12 :Float32;

  enum SensorSource {
    android @0;
    iOS @1;
    car @2;
    velodyne @3;  # Velodyne IMU
    fusion @4;
    external @5;
    ublox @6;
    trimble @7;
    qcomdiag @8;
  }
}

struct PeripheralState {
  pandaType @0 :PandaState.PandaType;
  voltage @1 :UInt32;
  current @2 :UInt32;
  fanSpeedRpm @3 :UInt16;

  usbPowerModeDEPRECATED @4 :UsbPowerModeDEPRECATED;
  enum UsbPowerModeDEPRECATED @0xa8883583b32c9877 {
    none @0;
    client @1;
    cdp @2;
    dcp @3;
  }
}

struct PandaState @0xa7649e2575e4591e {
  ignitionLine @2 :Bool;
  gasInterceptorDetected @4 :Bool;
  rxBufferOverflow @7 :UInt32;
  txBufferOverflow @8 :UInt32;
  gmlanSendErrs @9 :UInt32;
  pandaType @10 :PandaType;
  ignitionCan @13 :Bool;
  faultStatus @15 :FaultStatus;
  powerSaveEnabled @16 :Bool;
  uptime @17 :UInt32;
  faults @18 :List(FaultType);
  harnessStatus @21 :HarnessStatus;
  heartbeatLost @22 :Bool;
  interruptLoad @25 :Float32;
  fanPower @28 :UInt8;

  # can health
  canState0 @29 :PandaCanState;
  canState1 @30 :PandaCanState;
  canState2 @31 :PandaCanState;

  # safety stuff
  controlsAllowed @3 :Bool;
  safetyRxInvalid @19 :UInt32;
  safetyTxBlocked @24 :UInt32;
  safetyModel @14 :Car.CarParams.SafetyModel;
  safetyParam @27 :UInt16;
  alternativeExperience @23 :Int16;
  safetyRxChecksInvalid @32 :Bool;

  enum FaultStatus {
    none @0;
    faultTemp @1;
    faultPerm @2;
  }

  enum FaultType {
    relayMalfunction @0;
    unusedInterruptHandled @1;
    interruptRateCan1 @2;
    interruptRateCan2 @3;
    interruptRateCan3 @4;
    interruptRateTach @5;
    interruptRateGmlan @6;
    interruptRateInterrupts @7;
    interruptRateSpiDma @8;
    interruptRateSpiCs @9;
    interruptRateUart1 @10;
    interruptRateUart2 @11;
    interruptRateUart3 @12;
    interruptRateUart5 @13;
    interruptRateUartDma @14;
    interruptRateUsb @15;
    interruptRateTim1 @16;
    interruptRateTim3 @17;
    registerDivergent @18;
    interruptRateKlineInit @19;
    interruptRateClockSource @20;
    interruptRateTick @21;
    interruptRateExti @22;
    interruptRateSpi @23;
    interruptRateUart7 @24;
    # Update max fault type in boardd when adding faults
  }

  enum PandaType @0x8a58adf93e5b3751 {
    unknown @0;
    whitePanda @1;
    greyPanda @2;
    blackPanda @3;
    pedal @4;
    uno @5;
    dos @6;
    redPanda @7;
    redPandaV2 @8;
    tres @9;
  }

  enum HarnessStatus {
    notConnected @0;
    normal @1;
    flipped @2;
  }

  startedSignalDetectedDEPRECATED @5 :Bool;
  voltageDEPRECATED @0 :UInt32;
  currentDEPRECATED @1 :UInt32;
  hasGpsDEPRECATED @6 :Bool;
  fanSpeedRpmDEPRECATED @11 :UInt16;
  usbPowerModeDEPRECATED @12 :PeripheralState.UsbPowerModeDEPRECATED;
  safetyParamDEPRECATED @20 :Int16;
  safetyParam2DEPRECATED @26 :UInt32;

  struct PandaCanState {
    busOff @0 :Bool;
    busOffCnt @1 :UInt32;
    errorWarning @2 :Bool;
    errorPassive @3 :Bool;
    lastError @4 :LecErrorCode;
    lastStoredError @5 :LecErrorCode;
    lastDataError @6 :LecErrorCode;
    lastDataStoredError @7 :LecErrorCode;
    receiveErrorCnt @8 :UInt8;
    transmitErrorCnt @9 :UInt8;
    totalErrorCnt @10 :UInt32;
    totalTxLostCnt @11 :UInt32;
    totalRxLostCnt @12 :UInt32;
    totalTxCnt @13 :UInt32;
    totalRxCnt @14 :UInt32;
    totalFwdCnt @15 :UInt32;
    canSpeed @16 :UInt16;
    canDataSpeed @17 :UInt16;
    canfdEnabled @18 :Bool;
    brsEnabled @19 :Bool;
    canfdNonIso @20 :Bool;

    enum LecErrorCode {
      noError @0;
      stuffError @1;
      formError @2;
      ackError @3;
      bit1Error @4;
      bit0Error @5;
      crcError @6;
      noChange @7;
    }
  }
}

struct DriverState {
  frameId @0 :UInt32;
  modelExecutionTime @1 :Float32;
  dspExecutionTime @2 :Float32;
  rawPredictions @3 :Data;

  faceOrientation @4 :List(Float32);
  facePosition @5 :List(Float32);
  faceProb @6 :Float32;
  leftEyeProb @7 :Float32;
  rightEyeProb @8 :Float32;
  leftBlinkProb @9 :Float32;
  rightBlinkProb @10 :Float32;
  faceOrientationStd @11 :List(Float32);
  facePositionStd @12 :List(Float32);
  sunglassesProb @13 :Float32;
  poorVision @14 :Float32;
  partialFace @15 :Float32;
  distractedPose @16 :Float32;
  distractedEyes @17 :Float32;
  eyesOnRoad @18 :Float32;
  phoneUse @19 :Float32;
  occludedProb @20 :Float32;

  readyProb @21 :List(Float32);
  notReadyProb @22 :List(Float32);
}

struct DriverMonitoringState {
  events @0 :List(Car.CarEvent);
  faceDetected @1 :Bool;
  isDistracted @2 :Bool;
  distractedType @3 :UInt32;
  awarenessStatus @4 :Float32;
  posePitchOffset @5 :Float32;
  posePitchValidCount @6 :UInt32;
  poseYawOffset @7 :Float32;
  poseYawValidCount @8 :UInt32;
  stepChange @9 :Float32;
  awarenessActive @10 :Float32;
  awarenessPassive @11 :Float32;
  isLowStd @12 :Bool;
  hiStdCount @13 :UInt32;
  isActiveMode @14 :Bool;
}

struct DeviceState {
  usbOnline @0 :Bool;
  networkType @1 :NetworkType;
  networkInfo @2 :NetworkInfo;
  networkStrength @3 :NetworkStrength;
  networkMetered @4 :Bool;
  lastAthenaPingTime @5 :UInt64;

  started @6 :Bool;
  startedMonoTime @7 :UInt64;

  # system utilization
  freeSpacePercent @8 :Float32;
  memoryUsagePercent @9 :Int8;
  gpuUsagePercent @10 :Int8;
  cpuUsagePercent @11 :List(Int8);  # per-core cpu usage

  # power
  batteryPercent @12 :Int16;
  batteryCurrent @13 :Int32;
  chargingError @14 :Bool;
  chargingDisabled @15 :Bool;
  offroadPowerUsageUwh @16 :UInt32;
  carBatteryCapacityUwh @17 :UInt32;
  powerDrawW @18 :Float32;

  # device thermals
  cpuTempC @19 :List(Float32);
  gpuTempC @20 :List(Float32);
  memoryTempC @21 :Float32;
  ambientTempC @22 :Float32;
  nvmeTempC @23 :List(Float32);
  modemTempC @24 :List(Float32);
  pmicTempC @25 :List(Float32);
  thermalZones @26 :List(ThermalZone);
  thermalStatus @27 :ThermalStatus;

  fanSpeedPercentDesired @28 :UInt16;
  screenBrightnessPercent @29 :Int8;

  struct ThermalZone {
    name @0 :Text;
    temp @1 :Float32;
  }

  enum ThermalStatus {
    green @0;
    yellow @1;
    red @2;
    danger @3;
  }

  enum NetworkType {
    none @0;
    wifi @1;
    cell2G @2;
    cell3G @3;
    cell4G @4;
    cell5G @5;
    ethernet @6;
  }

  enum NetworkStrength {
    unknown @0;
    poor @1;
    moderate @2;
    good @3;
    great @4;
  }

  struct NetworkInfo {
    technology @0 :Text;
    operator @1 :Text;
    band @2 :Text;
    channel @3 :UInt16;
    extra @4 :Text;
    state @5 :Text;
  }
}

struct ProcLog {
  cpuTimes @0 :List(CPUTimes);
  mem @1 :Mem;
  procs @2 :List(Process);

  struct Process {
    pid @0 :Int32;
    name @1 :Text;
    state @2 :UInt8;
    ppid @3 :Int32;

    cpuUser @4 :Float32;
    cpuSystem @5 :Float32;
    cpuChildrenUser @6 :Float32;
    cpuChildrenSystem @7 :Float32;
    priority @8 :Int64;
    nice @9 :Int32;
    numThreads @10 :Int32;
    startTime @11 :Float64;

    memVms @12 :UInt64;
    memRss @13 :UInt64;

    processor @14 :Int32;

    cmdline @15 :List(Text);
    exe @16 :Text;
  }

  struct CPUTimes {
    cpuNum @0 :Int64;
    user @1 :Float32;
    nice @2 :Float32;
    system @3 :Float32;
    idle @4 :Float32;
    iowait @5 :Float32;
    irq @6 :Float32;
    softirq @7 :Float32;
  }

  struct Mem {
    total @0 :UInt64;
    free @1 :UInt64;
    available @2 :UInt64;
    buffers @3 :UInt64;
    cached @4 :UInt64;
    active @5 :UInt64;
    inactive @6 :UInt64;
    shared @7 :UInt64;
  }
}

struct LiveLocationKalman {

  # More info on reference frames:
  # https://github.com/commaai/openpilot/tree/master/common/transformations

  positionECEF @0 : Measurement;
  positionGeodetic @1 : Measurement;
  velocityECEF @2 : Measurement;
  velocityNED @3 : Measurement;
  velocityDevice @4 : Measurement;
  accelerationDevice @5: Measurement;


  # These angles are all eulers and roll, pitch, yaw
  # orientationECEF transforms to rot matrix: ecef_from_device
  orientationECEF @6 : Measurement;
  calibratedOrientationECEF @20 : Measurement;
  orientationNED @7 : Measurement;
  angularVelocityDevice @8 : Measurement;

  # orientationNEDCalibrated transforms to rot matrix: NED_from_calibrated
  calibratedOrientationNED @9 : Measurement;

  # Calibrated frame is simply device frame
  # aligned with the vehicle
  velocityCalibrated @10 : Measurement;
  accelerationCalibrated @11 : Measurement;
  angularVelocityCalibrated @12 : Measurement;

  gpsWeek @13 :Int32;
  gpsTimeOfWeek @14 :Float64;
  status @15 :Status;
  unixTimestampMillis @16 :Int64;
  inputsOK @17 :Bool = true;
  posenetOK @18 :Bool = true;
  gpsOK @19 :Bool = true;
  sensorsOK @21 :Bool = true;
  deviceStable @22 :Bool = true;
  timeSinceReset @23 :Float64;
  excessiveResets @24 :Bool;
  timeToFirstFix @25 :Float32;

  enum Status {
    uninitialized @0;
    uncalibrated @1;
    valid @2;
  }

  struct Measurement {
    value @0 : List(Float64);
    std @1 : List(Float64);
    valid @2 : Bool;
  }
}

struct ManagerState {
  processes @0 :List(ProcessState);

  struct ProcessState {
    name @0 :Text;
    pid @1 :Int32;
    running @2 :Bool;
    shouldBeRunning @4 :Bool;
    exitCode @3 :Int32;
  }
}

struct LiveTracks {
  trackId @0 :Int32;
  dRel @1 :Float32;
  yRel @2 :Float32;
  vRel @3 :Float32;
  aRel @4 :Float32;
  timeStamp @5 :Float32;
  status @6 :Float32;
  currentTime @7 :Float32;
  stationary @8 :Bool;
  oncoming @9 :Bool;
}

struct LiveParametersData {
  valid @0 :Bool;
  gyroBias @1 :Float32;
  angleOffsetDeg @2 :Float32;
  angleOffsetAverageDeg @3 :Float32;
  stiffnessFactor @4 :Float32;
  steerRatio @5 :Float32;
  sensorValid @6 :Bool;
  yawRate @7 :Float32;
  posenetSpeed @8 :Float32;
  posenetValid @9 :Bool;
  angleOffsetFastStd @10 :Float32;
  angleOffsetAverageStd @11 :Float32;
  stiffnessFactorStd @12 :Float32;
  steerRatioStd @13 :Float32;
  roll @14 :Float32;
}

struct UploaderState {
  immediateQueueSize @0 :UInt32;
  immediateQueueCount @1 :UInt32;
  rawQueueSize @2 :UInt32;
  rawQueueCount @3 :UInt32;

  # stats for last successfully uploaded file
  lastTime @4 :Float32;  # s
  lastSpeed @5 :Float32; # MB/s
  lastFilename @6 :Text;
}

struct GnssMeasurements {
  measTime @0 :UInt64;
  gpsWeek @1 :Int16;
  gpsTimeOfWeek @2 :Float64;

  correctedMeasurements @3 :List(CorrectedMeasurement);

  kalmanPositionECEF @4 :LiveLocationKalman.Measurement;
  kalmanVelocityECEF @5 :LiveLocationKalman.Measurement;
  positionECEF @6 :LiveLocationKalman.Measurement;
  velocityECEF @7 :LiveLocationKalman.Measurement;
  # Todo sync this with timing pulse of ublox

  struct CorrectedMeasurement {
    constellationId @0 :ConstellationId;
    svId @1 :UInt8;
    # Is 0 when not Glonass constellation.
    glonassFrequency @2 :Int8;
    pseudorange @3 :Float64;
    pseudorangeStd @4 :Float64;
    pseudorangeRate @5 :Float64;
    pseudorangeRateStd @6 :Float64;
    # Satellite position and velocity [x,y,z]
    satPos @7 :List(Float64);
    satVel @8 :List(Float64);
    ephemerisSource @9 :EphemerisSource;
  }

  struct EphemerisSource {
    type @0 :EphemerisSourceType;
    # first epoch in file:
    gpsWeek @1 :Int16; # -1 if Nav
    gpsTimeOfWeek @2 :Int32; # -1 if Nav. Integer for seconds is good enough for logs.
  }

  enum ConstellationId {
    # Satellite Constellation using the Ublox gnssid as index
    gps @0;
    sbas @1;
    galileo @2;
    beidou @3;
    imes @4;
    qznss @5;
    glonass @6;
  }

  enum EphemerisSourceType {
    nav @0;
    # Different ultra-rapid files:
    nasaUltraRapid @1;
    glonassIacUltraRapid @2;
    qcom @3;
  }
}

struct UbloxGnss {
  union {
    measurementReport @0 :MeasurementReport;
    ephemeris @1 :Ephemeris;
    ionoData @2 :IonoData;
    hwStatus @3 :HwStatus;
    hwStatus2 @4 :HwStatus2;
  }

  struct MeasurementReport {
    #received time of week in gps time in seconds and gps week
    rcvTow @0 :Float64;
    gpsWeek @1 :UInt16;
    # leap seconds in seconds
    leapSeconds @2 :UInt16;
    # receiver status
    receiverStatus @3 :ReceiverStatus;
    # num of measurements to follow
    numMeas @4 :UInt8;
    measurements @5 :List(Measurement);

    struct ReceiverStatus {
      # leap seconds have been determined
      leapSecValid @0 :Bool;
      # Clock reset applied
      clkReset @1 :Bool;
    }

    struct Measurement {
      svId @0 :UInt8;
      trackingStatus @1 :TrackingStatus;
      # pseudorange in meters
      pseudorange @2 :Float64;
      # carrier phase measurement in cycles
      carrierCycles @3 :Float64;
      # doppler measurement in Hz
      doppler @4 :Float32;
      # GNSS id, 0 is gps
      gnssId @5 :UInt8;
      glonassFrequencyIndex @6 :UInt8;
      # carrier phase locktime counter in ms
      locktime @7 :UInt16;
      # Carrier-to-noise density ratio (signal strength) in dBHz
      cno @8 :UInt8;
      # pseudorange standard deviation in meters
      pseudorangeStdev @9 :Float32;
      # carrier phase standard deviation in cycles
      carrierPhaseStdev @10 :Float32;
      # doppler standard deviation in Hz
      dopplerStdev @11 :Float32;
      sigId @12 :UInt8;

      struct TrackingStatus {
        # pseudorange valid
        pseudorangeValid @0 :Bool;
        # carrier phase valid
        carrierPhaseValid @1 :Bool;
        # half cycle valid
        halfCycleValid @2 :Bool;
        # half cycle subtracted from phase
        halfCycleSubtracted @3 :Bool;
      }
    }
  }

  struct Ephemeris {
    # This is according to the rinex (2?) format
    svId @0 :UInt16;
    year @1 :UInt16;
    month @2 :UInt16;
    day @3 :UInt16;
    hour @4 :UInt16;
    minute @5 :UInt16;
    second @6 :Float32;
    af0 @7 :Float64;
    af1 @8 :Float64;
    af2 @9 :Float64;

    iode @10 :Float64;
    crs @11 :Float64;
    deltaN @12 :Float64;
    m0 @13 :Float64;

    cuc @14 :Float64;
    ecc @15 :Float64;
    cus @16 :Float64;
    a @17 :Float64; # note that this is not the root!!

    toe @18 :Float64;
    cic @19 :Float64;
    omega0 @20 :Float64;
    cis @21 :Float64;

    i0 @22 :Float64;
    crc @23 :Float64;
    omega @24 :Float64;
    omegaDot @25 :Float64;

    iDot @26 :Float64;
    codesL2 @27 :Float64;
    gpsWeek @28 :Float64;
    l2 @29 :Float64;

    svAcc @30 :Float64;
    svHealth @31 :Float64;
    tgd @32 :Float64;
    iodc @33 :Float64;

    transmissionTime @34 :Float64;
    fitInterval @35 :Float64;

    toc @36 :Float64;

    ionoCoeffsValid @37 :Bool;
    ionoAlpha @38 :List(Float64);
    ionoBeta @39 :List(Float64);

  }

  struct IonoData {
    svHealth @0 :UInt32;
    tow  @1 :Float64;
    gpsWeek @2 :Float64;

    ionoAlpha @3 :List(Float64);
    ionoBeta @4 :List(Float64);

    healthValid @5 :Bool;
    ionoCoeffsValid @6 :Bool;
  }

  struct HwStatus {
    noisePerMS @0 :UInt16;
    agcCnt @1 :UInt16;
    aStatus @2 :AntennaSupervisorState;
    aPower @3 :AntennaPowerStatus;
    jamInd @4 :UInt8;
    flags @5 :UInt8;

    enum AntennaSupervisorState {
      init @0;
      dontknow @1;
      ok @2;
      short @3;
      open @4;
    }

    enum AntennaPowerStatus {
      off @0;
      on @1;
      dontknow @2;
    }
  }

  struct HwStatus2 {
    ofsI @0 :Int8;
    magI @1 :UInt8;
    ofsQ @2 :Int8;
    magQ @3 :UInt8;
    cfgSource @4 :ConfigSource;
    lowLevCfg @5 :UInt32;
    postStatus @6 :UInt32;

    enum ConfigSource {
      undefined @0;
      rom @1;
      otp @2;
      configpins @3;
      flash @4;
    }
  }
}

struct ModelRaw {
  frameId @0 :UInt32;
  frameIdExtra @1 :UInt32;
  frameAge @2 :UInt32;
  frameDropPerc @3 :Float32;
  timestampEof @4 :UInt64;
  modelExecutionTime @5 :Float32;
  valid @6 :Bool;

  rawPredictions @7 :List(Float32);
}

struct Clocks {
  bootTimeNanos @0 :UInt64;
  monotonicNanos @1 :UInt64;
  monotonicRawNanos @2 :UInt64;
  wallTimeNanos @3 :UInt64;
  modemUptimeMillis @4 :UInt64;
}

struct Joystick {
  # convenient for debug and live tuning
  axes @0: List(Float32);
  buttons @1: List(Bool);
}

struct LiveTorqueParametersData {
  liveValid @0 :Bool;
  latAccelFactorRaw @1 :Float32;
  latAccelOffsetRaw @2 :Float32;
  frictionCoefficientRaw @3 :Float32;
  latAccelFactorFiltered @4 :Float32;
  latAccelOffsetFiltered @5 :Float32;
  frictionCoefficientFiltered @6 :Float32;
  totalBucketPoints @7 :Float32;
  decay @8 :Float32;
  maxResets @9 :Float32;
  points @10 :List(List(Float32));
  version @11 :Int32;
  useParams @12 :Bool;
}

struct Event {
  logMonoTime @33 :UInt64;  # nanoseconds
  valid @34 :Bool = true;

  union {
    procLog @0 :ProcLog;
    roadCameraState@1: FrameData;
    accelerometer @2: SensorEventData;
    gyroscope @3: SensorEventData;
    desireDEPRECATED @4: Desire;
    modelV2 @5: ModelDataV2;
    liveCalibration @6: LiveCalibrationData;
    cameraOdometry @7: CameraOdometry;
    carState @8 :Car.CarState;
    carControl @9 :Car.CarControl;
    can @10 :List(CanData);
    sendcan @11 :List(CanData);
    lateralPlan @12 :LateralPlan;
    carParams @13: Car.CarParams;
    controlsState @14 :ControlsState;
    radarState @15 :RadarState;
    longitudinalPlan @16 :LongitudinalPlan;
    carEvents @17: List(Car.CarEvent);
    frameDataDEPRECATED@18: FrameData;
    sensorEventsDEPRECATED @19 :List(SensorEventData);
    gpsLocationExternal @20 :GpsLocationData;
    peripheralState @21 :PeripheralState;
    pandaStates @22 :List(PandaState);
    driverState @23 :DriverState;
    driverMonitoringState @24: DriverMonitoringState;
    driverCameraState@25: FrameData;
    deviceState @26 :DeviceState;
    liveLocationKalman @27 :LiveLocationKalman;
    managerState @28 :ManagerState;
    liveParameters @29 :LiveParametersData;
    wideRoadCameraState@30: FrameData;
    logMessage @31 :Text;
    errorLogMessage @32 :Text;
    initData @35 :InitData;
    ubloxRaw @36 :Data;
    liveTracks @37 :List(LiveTracks);
    sentinel @38 :Sentinel;
    uploaderState @39 :UploaderState;
    gnssMeasurements @40 :GnssMeasurements;
    ubloxGnss @41 :UbloxGnss;
    gyroscope2 @42 :SensorEventData;
    accelerometer2 @43 :SensorEventData;
    gpsLocation @44 :GpsLocationData;
    navModel @45 :NavModelData;
    modelRaw @46 :ModelRaw;
    roadCameraBuffer @47 :FrameBuffer;
    wideRoadCameraBuffer @48 :FrameBuffer;
    driverCameraBuffer @49 :FrameBuffer;
    testJoystick @50 :Joystick;
    clocks @51 :Clocks;
    liveTorqueParameters @52 :LiveTorqueParametersData;
  }
}
