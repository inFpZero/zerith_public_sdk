import logging
from typing import Any

EVENT_LEVEL: int
EVENT: int

class EventLogger(logging.Logger):
    def event(self, message: str, *args: Any, **kwargs: Any) -> None: ...

logger: EventLogger
