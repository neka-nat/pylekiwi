"""Host-clock command leases. Call under the host's motor lock."""

import time
from dataclasses import dataclass
from uuid import uuid4

from pylekiwi.models import CommandLease, CommandLeaseRequest, LekiwiCommand


@dataclass
class _ClientLease:
    lease: CommandLease
    sequence: int = -1


class CommandGuard:
    def __init__(
        self,
        *,
        max_validity_s: float = 5.0,
        base_max_validity_s: float = 0.5,
        clock=time.monotonic_ns,
        max_clients: int = 256,
    ):
        self.host_id = str(uuid4())
        self.max_validity_s = max_validity_s
        self.base_max_validity_s = base_max_validity_s
        self.clock = clock
        self.max_clients = max_clients
        self._clients: dict[str, _ClientLease] = {}

    def issue(self, request: CommandLeaseRequest) -> CommandLease:
        now = self.clock()
        if request.validity_s > self.max_validity_s:
            raise ValueError("Requested command validity exceeds the host limit.")
        self._clients = {
            key: value
            for key, value in self._clients.items()
            if value.lease.expires_at_monotonic_ns > now
        }
        previous = self._clients.get(request.client_id)
        if previous is None and len(self._clients) >= self.max_clients:
            raise ValueError("Too many active command clients.")
        lease = CommandLease(
            host_id=self.host_id,
            client_id=request.client_id,
            lease_id=str(uuid4()),
            issued_at_monotonic_ns=now,
            expires_at_monotonic_ns=now + int(request.validity_s * 1e9),
        )
        self._clients[request.client_id] = _ClientLease(
            lease, previous.sequence if previous else -1
        )
        return lease

    def validate(self, command: LekiwiCommand, *, reserve: bool = True) -> int:
        envelope = command.envelope
        if envelope is None:
            raise ValueError(
                "Command lease required; update the client and host together."
            )
        client = self._clients.get(envelope.lease.client_id)
        if client is None or envelope.lease != client.lease:
            raise ValueError("Unknown, replaced, or previous-host command lease.")
        lease = client.lease
        if self.clock() >= lease.expires_at_monotonic_ns:
            raise ValueError("Command lease has expired.")
        if command.base_command is not None and (
            lease.expires_at_monotonic_ns - lease.issued_at_monotonic_ns
            > int(self.base_max_validity_s * 1e9)
        ):
            raise ValueError("Base command lease exceeds the base validity limit.")
        if reserve:
            if envelope.sequence <= client.sequence:
                raise ValueError("Duplicate or out-of-order command.")
            client.sequence = envelope.sequence
        elif envelope.sequence != client.sequence:
            raise ValueError("Command was superseded during preparation.")
        return lease.expires_at_monotonic_ns
