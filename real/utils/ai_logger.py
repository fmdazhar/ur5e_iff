import logging

import colorlog


class Logger:
    """
    A logger class.

    Args:
        log_level (str): the following modes are supported:
            `debug`, `info`, `warn`, `error`, `critical`.
    """

    def __init__(self, log_level='debug'):
        formatter = colorlog.ColoredFormatter(
            "%(log_color)s[%(levelname)s]%(reset)s[%(asctime)s]: "
            "%(message_log_color)s%(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
            reset=True,
            log_colors={
                'DEBUG': 'cyan',
                'INFO': 'green',
                'WARNING': 'yellow',
                'ERROR': 'red',
                'CRITICAL': 'red,bg_white',
            },
            secondary_log_colors={
                'message': {
                    'DEBUG': 'cyan',
                    'INFO': 'green',
                    'WARNING': 'yellow',
                    'ERROR': 'red',
                    'CRITICAL': 'red'
                }
            },
            style='%'
        )

        handler = colorlog.StreamHandler()
        handler.setFormatter(formatter)

        self.logger = colorlog.getLogger('AIRobot')
        self.logger.addHandler(handler)
        self.set_log_level(log_level)

    def log_debug(self, msg):
        """
        Logging debug information

        Args:
            msg (str): message to log
        """
        self.logger.debug(msg)

    def log_info(self, msg):
        """
        Logging info information

        Args:
            msg (str): message to log
        """
        self.logger.info(msg)

    def log_warning(self, msg):
        """
        Logging warning information

        Args:
            msg (str): message to log
        """
        self.logger.warning(msg)

    def log_error(self, msg):
        """
        Logging error information

        Args:
            msg (str): message to log
        """
        self.logger.error(msg)

    def log_critical(self, msg):
        """
        Logging critical information

        Args:
            msg (str): message to log
        """
        self.logger.critical(msg)

    def set_log_level(self, log_level):
        """
        Set logging level

        Args:
            log_level (str): the following modes are supported:
                `debug`, `info`, `warn`, `error`, `critical`

        """
        if 'debug' in log_level:
            self.log_level = logging.DEBUG
        elif 'info' in log_level:
            self.log_level = logging.INFO
        elif 'warn' in log_level:
            self.log_level = logging.WARNING
        elif 'error' in log_level:
            self.log_level = logging.ERROR
        elif 'critical' in log_level:
            self.log_level = logging.CRITICAL
        else:
            raise ValueError('Unknown logging '
                             'level: %s' % log_level)
        self.logger.setLevel(self.log_level)


if __name__ == '__main__':
    ai_logger = Logger('debug')
    ai_logger.log_debug("A quirky message only developers care about")
    ai_logger.log_info("Curious users might want to know this")
    ai_logger.log_warning("Something is wrong and any user should be informed")
    ai_logger.log_error("Serious stuff, this is red for a reason")
    ai_logger.log_critical("OH NO everything is on fire")
