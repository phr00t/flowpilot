import os
import subprocess
import glob
import hashlib
import shutil
from common.basedir import BASEDIR
from selfdrive.swaglog import cloudlog
from common.spinner import Spinner
import time

android_packages = ("com.android.carrierconfig", "com.mixplorer", "com.google.android.inputmethod.korean", "com.gmd.hidesoftkeys", "com.android.chrome",)

def get_installed_apks():
  dat = subprocess.check_output(["pm", "list", "packages", "-f"], encoding='utf8').strip().split("\n")
  ret = {}
  for x in dat:
    if x.startswith("package:"):
      v, k = x.split("package:")[1].split("=")
      ret[k] = v
  return ret

def install_apk(path):
  # can only install from world readable path
  install_path = "/sdcard/%s" % os.path.basename(path)
  shutil.copyfile(path, install_path)

  ret = subprocess.call(["pm", "install", "-r", install_path])
  os.remove(install_path)
  return ret == 0

def appops_set(package, op, mode):
  system(f"LD_LIBRARY_PATH= appops set {package} {op} {mode}")

def pm_grant(package, permission):
  system(f"pm grant {package} {permission}")

def system(cmd):
  try:
    cloudlog.info("running %s" % cmd)
    subprocess.check_output(cmd, stderr=subprocess.STDOUT, shell=True)
  except subprocess.CalledProcessError as e:
    cloudlog.event("running failed",
      cmd=e.cmd,
      output=e.output[-1024:],
      returncode=e.returncode)

# *** external functions ***

def update_apks(show_spinner=False):
  # install apks
  installed = get_installed_apks()

  install_apks = glob.glob(os.path.join(BASEDIR, "selfdrive/assets/addon/apk/*.apk"))
  if show_spinner:
    spinner = Spinner()

  show_spinner = False
  for apk in install_apks:
    app = os.path.basename(apk)[:-4]
    if app not in installed:
      installed[app] = None

  cloudlog.info("installed apks %s" % (str(installed), ))

  for app in installed.keys():
    apk_path = os.path.join(BASEDIR, "selfdrive/assets/addon/apk/"+app+".apk")
    if not os.path.exists(apk_path):
      continue

    h1 = hashlib.sha1(open(apk_path, 'rb').read()).hexdigest()
    h2 = None
    if installed[app] is not None:
      h2 = hashlib.sha1(open(installed[app], 'rb').read()).hexdigest()
      cloudlog.info("comparing version of %s  %s vs %s" % (app, h1, h2))

    if h2 is None or h1 != h2:
      show_spinner = True
      spinner.update("installing %s" % app)
      cloudlog.info("installing %s" % app)

      success = install_apk(apk_path)
      if not success:
        cloudlog.info("needing to uninstall %s" % app)
        system("pm uninstall %s" % app)
        success = install_apk(apk_path)

      if app == "com.mixplorer":
        appops_set("com.mixplorer", "SU", "allow")
        pm_grant("com.mixplorer", "android.permission.READ_EXTERNAL_STORAGE")
      if app == "com.google.android.inputmethod.korean":
        pm_grant("com.google.android.inputmethod.korean", "android.permission.BIND_INPUT_METHOD")
        system("am start com.google.android.inputmethod.korean/com.google.android.apps.inputmethod.libs.framework.core.LauncherActivity")
        time.sleep(3)
        system("pkill com.google.android.inputmethod.korean")        
        system("settings put secure enabled_input_methods com.google.android.inputmethod.korean/.KoreanIme")
        system("settings put secure default_input_method com.google.android.inputmethod.korean/.KoreanIme")
        system("cp -f /data/openpilot/selfdrive/assets/addon/param/com.google.android.inputmethod.korean*.xml /data/data/com.google.android.inputmethod.korean/shared_prefs/")
        time.sleep(1)
        system("am start com.google.android.inputmethod.korean/com.google.android.apps.inputmethod.libs.framework.core.LauncherActivity")
        time.sleep(3)
        system("reboot")
      if app == "com.gmd.hidesoftkeys":
        appops_set("com.gmd.hidesoftkeys", "SU", "allow")
        pm_grant("com.gmd.hidesoftkeys", "android.permission.SYSTEM_ALERT_WINDOW")
        system("am start com.gmd.hidesoftkeys/com.gmd.hidesoftkeys.MainActivity")
        time.sleep(6)
        system("pkill com.gmd.hidesoftkeys")
        time.sleep(1)
        system("cp -f /data/openpilot/selfdrive/assets/addon/param/com.gmd.hidesoftkeys_preferences.xml /data/data/com.gmd.hidesoftkeys/shared_prefs/")
        system("cp -f /data/openpilot/selfdrive/assets/addon/param/appops.xml /data/system/")
        time.sleep(1)
        system("am start com.gmd.hidesoftkeys/com.gmd.hidesoftkeys.MainActivity")
        time.sleep(5)
        system("reboot")

      assert success

  if show_spinner:
    spinner.close()

def pm_apply_packages(cmd):
  for p in android_packages:
    system("pm %s %s" % (cmd, p))

if __name__ == "__main__":
  update_apks()
