import logging
import sys
import cereal.messaging as messaging

PyLog = ''

def AddLog(log):
    global PyLog

    print('Adding: ' + log)

    PyLog = log + " "

def checkIfPyLog(pm):
    global PyLog

    print("Checking PyLog, which is: " + PyLog)

    if len(PyLog) > 0:

        print("Pylog got: " + PyLog)

        msg = messaging.new_message("uploaderState")
        us = msg.uploaderState
        us.lastFilename = PyLog
        pm.send("uploaderState", msg)
        PyLog = ''

def get_logger(name, file_name=None, level=logging.DEBUG):
    logger = logging.getLogger(name)
    logger.setLevel(level)
    formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(formatter)
    logger.handlers.clear()
    logger.addHandler(sh)
    if file_name:
        fh = logging.FileHandler(file_name)
        fh.setFormatter(formatter)
        logger.addHandler(fh)
    return logger
