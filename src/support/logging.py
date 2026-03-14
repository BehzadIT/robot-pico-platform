"""
Convenience module for logging.

Provides easy access to all logging functions with a clean import:

# Option 1: Import specific log levels
from log import logd, logi, logw, loge, logv

# Option 2: Import all log levels at once
from log import *

# Option 3: Use the grouped logger
from log import log
log.d("Debug message")
log.i("Info message")
"""

from src.support.logger import (
    logd,  # Debug
    logi,  # Info
    logw,  # Warning
    loge,  # Error
    logv,  # Verbose
    telemetry,
    enable_uart_print_mirror,
    disable_stdout_uart_mirror,
    set_global_log_level,
    set_tag_log_level,
)

# Create a logger group for dot notation access
class LoggerGroup:
    """Grouped logger for more concise logging.
    
    Example:
        from log import log
        log.d("Debug message")
        log.i("Info message")
        log.w("Warning message")
        log.e("Error message")
        log.v("Verbose message")
    """
    def __init__(self):
        self.d = logd
        self.i = logi
        self.w = logw
        self.e = loge
        self.v = logv

# Create a default logger instance
log = LoggerGroup()

# Enable UART mirroring by default
enable_uart_print_mirror()

__all__ = [
    'logd', 'logi', 'logw', 'loge', 'logv',  # Individual loggers
    'log',  # Grouped logger
    'telemetry',
    'enable_uart_print_mirror',
    'disable_stdout_uart_mirror',
    'set_global_log_level',
    'set_tag_log_level',
    'LoggerGroup'
]
