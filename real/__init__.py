from utils.ai_logger import Logger


logger = Logger('debug')


def set_log_level(log_level):
    """
    Set logging level

    Args:
        log_level (str): one of: 'debug', 'info',
            'warn', 'error', 'critical'
    """
    logger.set_level(log_level)


def log_warn(msg):
    """
    Logging warning information

    Args:
        msg (str): message to log
    """
    logger.warning(msg)


def log_info(msg):
    """
    Logging info information

    Args:
        msg (str): message to log
    """
    logger.info(msg)


def log_error(msg):
    """
    Logging error information

    Args:
        msg (str): message to log
    """
    logger.error(msg)


def log_debug(msg):
    """
    Logging debug information

    Args:
        msg (str): message to log
    """
    logger.debug(msg)


def log_critical(msg):
    """
    Logging critical information

    Args:
        msg (str): message to log
    """
    logger.critical(msg)
