"""Lightweight in-process publish/subscribe broker."""
from __future__ import annotations

import logging
from collections import defaultdict
from typing import Callable, DefaultDict, Dict, List

LOGGER = logging.getLogger(__name__)

Handler = Callable[[Dict[str, object]], None]

_subscribers: DefaultDict[str, List[Handler]] = defaultdict(list)


def subscribe(topic: str, handler: Handler) -> None:
    LOGGER.debug("Subscriber added for topic %s: %s", topic, handler)
    _subscribers[topic].append(handler)


def publish(topic: str, payload: Dict[str, object]) -> None:
    LOGGER.debug("Publishing to %s: %s", topic, payload)
    for handler in list(_subscribers.get(topic, [])):
        try:
            handler(payload)
        except Exception as exc:  # pragma: no cover - safety logging
            LOGGER.exception("Handler %s failed for topic %s: %s", handler, topic, exc)


def reset() -> None:
    _subscribers.clear()


__all__ = ["publish", "subscribe", "reset"]
