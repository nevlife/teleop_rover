import json

import zenoh

from teleop_contracts import make_envelope


def _split_csv(s: str) -> list[str]:
    """Split a comma-separated locator string into a list of trimmed entries."""
    return [tok.strip() for tok in s.split(",") if tok.strip()]


class ZenohTransport:
    """Single-session Zenoh transport for the telemetry plane.

    `locator` accepts either:
      * a single string  ("tcp/host:7447")
      * a comma-separated string  ("tcp/host-a:7447,tcp/host-b:7447")
      * a list/tuple of strings  (passed straight through as endpoints)

    All endpoints are joined as a JSON array on `connect/endpoints` so Zenoh
    will fail over within ONE session when a link dies. This is the Task-6
    multi-endpoint behaviour for telemetry; hot-replication for video lives in
    the C++ bridge.
    """

    def __init__(self, locator, logger):
        self._logger = logger
        self._pubs: dict[str, zenoh.Publisher] = {}
        self._subs: list = []

        endpoints: list[str] = []
        if isinstance(locator, (list, tuple)):
            for entry in locator:
                endpoints.extend(_split_csv(str(entry)))
        elif isinstance(locator, str):
            endpoints = _split_csv(locator)

        conf = zenoh.Config()
        if endpoints:
            conf.insert_json5("connect/endpoints", json.dumps(endpoints))

        try:
            self._session = zenoh.open(conf)
        except Exception as e:
            logger.fatal(f"Zenoh connect failed: {e}")
            raise RuntimeError(f"Zenoh connect failed: {e}") from e

        summary = ",".join(endpoints) if endpoints else "auto-discovery"
        logger.info(f"Zenoh connected -> {summary}")

    @property
    def session(self) -> zenoh.Session:
        return self._session

    def declare_publisher(self, key: str, **qos) -> None:
        existing = self._pubs.pop(key, None)
        if existing is not None:
            try:
                existing.undeclare()
            except Exception as e:
                self._logger.warning(f"Error undeclaring old publisher [{key}]: {e}")
        self._pubs[key] = self._session.declare_publisher(key, **qos)

    def declare_subscriber(self, key: str, callback) -> None:
        self._subs.append(self._session.declare_subscriber(key, callback))

    def put(self, key: str, data) -> None:
        """Publish ``data`` on ``key`` as a JSON envelope.

        ``data`` must be a dict (or list — see below). All publishes are
        wrapped in ``make_envelope(...)`` so every JSON message on the
        wire carries the ``{v, ts, ...}`` envelope. The legacy ``gpu``
        payload was a bare list; it is now wrapped under a ``gpus`` key
        upstream in :class:`TelemetrySerializer` so the envelope contract
        holds uniformly.
        """
        try:
            if isinstance(data, dict):
                payload = make_envelope(data)
            else:
                # Defensive: bare list/scalar publish would break the
                # envelope. Wrap under "data" so older non-dict callers
                # don't silently bypass the envelope.
                payload = make_envelope({"data": data})
            self._pubs[key].put(json.dumps(payload))
        except Exception as e:
            self._logger.warning(f"zenoh put [{key}]: {e}")

    def close(self) -> None:
        for sub in self._subs:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subs.clear()
        for key, pub in self._pubs.items():
            try:
                pub.undeclare()
            except Exception as e:
                self._logger.warning(f"Error undeclaring publisher [{key}]: {e}")
        self._pubs.clear()
        try:
            self._session.close()
        except Exception:
            pass
