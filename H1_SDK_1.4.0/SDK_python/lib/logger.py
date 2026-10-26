"""H1 Python logger configured by ROBOT_LOG_MODULE before import."""

import logging
import os
import re
from datetime import datetime
from logging.handlers import RotatingFileHandler


# EVENT 位于 INFO 和 WARNING 之间，保持标准 logging.LogRecord 语义。
EVENT_LEVEL = logging.INFO + 5
EVENT = EVENT_LEVEL
logging.addLevelName(EVENT_LEVEL, "EVENT")


def _event(self, message, *args, **kwargs):
    """记录重要事件，参数格式与 logging.Logger.info 一致。"""
    if self.isEnabledFor(EVENT_LEVEL):
        # 日志位置指向 logger.event() 的业务调用点。
        kwargs.setdefault("stacklevel", 2)
        self._log(EVENT_LEVEL, message, args, **kwargs)


if not hasattr(logging.Logger, "event"):
    logging.Logger.event = _event


class ExactLevelFilter(logging.Filter):
    """仅允许指定等级进入 EVENT 专用文件。"""

    def __init__(self, level):
        super().__init__()
        self.level = level

    def filter(self, record):
        return record.levelno == self.level


class DailyRotatingSizeHandler(RotatingFileHandler):
    """同时支持按日期、文件大小轮转和保留天数清理。"""

    def __init__(self, log_dir, prefix, max_bytes, backup_count, max_days=30):
        self.log_dir = log_dir
        self.prefix = prefix
        self.max_days = max_days
        self._emit_count = 0
        self.current_date = datetime.now().strftime("%Y-%m-%d")
        self.symlink_path = os.path.join(log_dir, "current.log")

        os.makedirs(log_dir, exist_ok=True)
        filename = os.path.join(log_dir, f"{prefix}_{self.current_date}.log")
        super().__init__(
            filename,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
        )
        self._update_symlink()

    def _update_symlink(self):
        try:
            if os.path.lexists(self.symlink_path):
                os.remove(self.symlink_path)
            os.symlink(os.path.basename(self.baseFilename), self.symlink_path)
        except OSError as error:
            print(f"Failed to create log symlink: {error}")

    def doRollover(self):
        now_date = datetime.now().strftime("%Y-%m-%d")
        if now_date != self.current_date:
            self.current_date = now_date
            self.baseFilename = os.path.join(
                self.log_dir, f"{self.prefix}_{self.current_date}.log"
            )
        super().doRollover()
        self._update_symlink()

    def emit(self, record):
        now_date = datetime.now().strftime("%Y-%m-%d")
        if now_date != self.current_date:
            self.current_date = now_date
            if self.stream:
                self.stream.close()
            self.baseFilename = os.path.join(
                self.log_dir, f"{self.prefix}_{self.current_date}.log"
            )
            self.stream = self._open()
            self._update_symlink()

        super().emit(record)
        self._emit_count += 1
        if self._emit_count >= 10:
            self.cleanup_retention()
            self._emit_count = 0

    def cleanup_retention(self):
        try:
            now = datetime.now()
            pattern = re.compile(
                rf"{re.escape(self.prefix)}_(\d{{4}}-\d{{2}}-\d{{2}})\.log"
            )
            for filename in os.listdir(self.log_dir):
                if filename == "current.log":
                    continue
                match = pattern.match(filename)
                if not match:
                    continue
                file_date = datetime.strptime(match.group(1), "%Y-%m-%d")
                if (now - file_date).days > self.max_days:
                    try:
                        os.remove(os.path.join(self.log_dir, filename))
                    except OSError:
                        pass
        except (OSError, ValueError) as error:
            print(f"Cleanup logs failed: {error}")


class ColoredFormatter(logging.Formatter):
    COLORS = {
        "DEBUG": "\033[36m",
        "INFO": "\033[32m",
        "EVENT": "\033[35m",
        "WARNING": "\033[33m",
        "ERROR": "\033[31m",
        "CRITICAL": "\033[41m",
    }
    RESET = "\033[0m"

    def format(self, record):
        color = self.COLORS.get(record.levelname, self.RESET)
        pattern = (
            f"{color}[%(asctime)s.%(msecs)03d] [%(levelname)s] "
            f"[%(name)s] [%(filename)s:%(lineno)d]{self.RESET} %(message)s"
        )
        return logging.Formatter(
            pattern, datefmt="%Y-%m-%d %H:%M:%S"
        ).format(record)


LOG_ROOT = os.getenv("ROBOT_LOG_ROOT", "/var/log/robot")
LOG_MODULE_NAME = os.getenv("ROBOT_LOG_MODULE", "H1_SDK")
LOG_DIR = os.path.join(LOG_ROOT, LOG_MODULE_NAME, "normal")
LOG_PREFIX = LOG_MODULE_NAME
EVENT_LOG_DIR = os.path.join(LOG_ROOT, LOG_MODULE_NAME, "event")
EVENT_LOG_PREFIX = f"{LOG_MODULE_NAME}_event"
MAX_LOG_SIZE = 100 * 1024 * 1024
BACKUP_COUNT = 10
MAX_DAYS = 30

logger = logging.getLogger(LOG_MODULE_NAME)
logger.setLevel(logging.INFO)
logger.propagate = False

# 防止 importlib.reload 或重复加载时重复添加 Handler。
if not getattr(logger, "_robot_log_configured", False):
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(ColoredFormatter())
    logger.addHandler(console_handler)

    file_formatter = logging.Formatter(
        "[%(asctime)s.%(msecs)03d] [%(levelname)s] "
        "[%(name)s] [%(filename)s:%(lineno)d] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    file_handler = DailyRotatingSizeHandler(
        LOG_DIR, LOG_PREFIX, MAX_LOG_SIZE, BACKUP_COUNT, MAX_DAYS
    )
    file_handler.setFormatter(file_formatter)
    logger.addHandler(file_handler)

    event_file_handler = DailyRotatingSizeHandler(
        EVENT_LOG_DIR,
        EVENT_LOG_PREFIX,
        MAX_LOG_SIZE,
        BACKUP_COUNT,
        MAX_DAYS,
    )
    event_file_handler.addFilter(ExactLevelFilter(EVENT_LEVEL))
    event_file_handler.setFormatter(file_formatter)
    logger.addHandler(event_file_handler)
    logger._robot_log_configured = True


__all__ = ["EVENT", "EVENT_LEVEL", "logger"]
