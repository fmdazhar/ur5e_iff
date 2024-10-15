import logging
import colorlog


class Logger:
    """
    A static logger class.

    Supported log levels: `debug`, `info`, `warn`, `error`, `critical`.
    """

    # Static method to initialize the logger configuration
    @staticmethod
    def _initialize_logger():
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

        logger = colorlog.getLogger('AIRobot')
        logger.addHandler(handler)

        # Set default log level to DEBUG
        logger.setLevel(logging.DEBUG)

        return logger

    # Static logger object
    _logger = _initialize_logger.__func__()

    @staticmethod
    def log_debug(msg):
        """
        Logging debug information.

        Args:
            msg (str): message to log
        """
        Logger._logger.debug(msg)

    @staticmethod
    def log_info(msg):
        """
        Logging info information.

        Args:
            msg (str): message to log
        """
        Logger._logger.info(msg)

    @staticmethod
    def log_warning(msg):
        """
        Logging warning information.

        Args:
            msg (str): message to log
        """
        Logger._logger.warning(msg)

    @staticmethod
    def log_error(msg):
        """
        Logging error information.

        Args:
            msg (str): message to log
        """
        Logger._logger.error(msg)

    @staticmethod
    def log_critical(msg):
        """
        Logging critical information.

        Args:
            msg (str): message to log
        """
        Logger._logger.critical(msg)

    @staticmethod
    def set_log_level(log_level):
        """
        Set logging level.

        Args:
            log_level (str): supported modes are `debug`, `info`, `warn`, `error`, `critical`
        """
        if log_level == 'debug':
            Logger._logger.setLevel(logging.DEBUG)
        elif log_level == 'info':
            Logger._logger.setLevel(logging.INFO)
        elif log_level == 'warn':
            Logger._logger.setLevel(logging.WARNING)
        elif log_level == 'error':
            Logger._logger.setLevel(logging.ERROR)
        elif log_level == 'critical':
            Logger._logger.setLevel(logging.CRITICAL)
        else:
            raise ValueError(f"Unknown logging level: {log_level}")


# Test the static logger
if __name__ == '__main__':
    Logger.log_debug("A quirky message only developers care about")
    Logger.log_info("Curious users might want to know this")
    Logger.log_warning("Something is wrong and any user should be informed")
    Logger.log_error("Serious stuff, this is red for a reason")
    Logger.log_critical("OH NO everything is on fire")
