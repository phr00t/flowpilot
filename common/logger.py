import logging
import sys
import cereal.messaging as messaging
import socket

class SocketLogger:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind(('', 9000))
        self.s.listen(10)
        self.connection, self.address = self.s.accept()

    def Send(self, log):
        self.s.send((log + "\n").encode())

sLogger = SocketLogger()

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
