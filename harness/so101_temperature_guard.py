"""Confirm transient feedback below the hardware limit without averaging away heat.

No new trajectory waypoint may be issued until this function returns. A motor
fault must be checked by the caller on every feedback packet. The confirmation
reader also checks current, voltage and position before returning a pair.
"""

import time


class TemperatureLimitError(RuntimeError):
    """A hardware limit or sustained software cutoff was reached."""


class TemperatureDataError(RuntimeError):
    """Feedback could not be validated; stop issuing motion goals."""


def confirm_temperature_sample(
    value, read_value, *, hardware_limit, software_limit, pause=time.sleep
):
    """Monitor full feedback packets; confirm warm samples without extra 1-byte reads."""
    if not 40 <= software_limit <= hardware_limit - 10 or hardware_limit > 100:
        raise RuntimeError("Invalid thermal protection settings")

    def hard_check(sample):
        if not isinstance(sample, int) or sample < 0 or sample > 255:
            raise TemperatureDataError("Invalid temperature feedback")
        if sample >= hardware_limit:
            raise TemperatureLimitError(
                f"Reported temperature {sample} C reached stop threshold {hardware_limit} C"
            )

    hard_check(value)
    if value < min(45, software_limit):
        return []
    confirmed = []
    for _ in range(2):
        pause(0.02)
        fresh = read_value()
        hard_check(fresh)
        confirmed.append(fresh)
        if fresh >= software_limit:
            raise TemperatureLimitError("High temperature persisted on confirmation")
    if abs(confirmed[0] - confirmed[1]) > 2:
        raise TemperatureDataError("Temperature confirmation did not stabilize")
    return confirmed


def confirm_temperature(
    pair, read_pair, *, hardware_limit, software_limit, pause=time.sleep
):
    if not 40 <= software_limit <= hardware_limit - 10 or hardware_limit > 100:
        raise RuntimeError("Invalid thermal protection settings")

    def hard_check(values):
        if len(values) != 2 or any(
            not isinstance(v, int) or v < 0 or v > 255 for v in values
        ):
            raise TemperatureDataError("Invalid temperature feedback")
        if max(values) >= hardware_limit:
            raise TemperatureLimitError(
                f"Temperature readings {list(values)} C include a value at or above stop threshold {hardware_limit} C"
            )

    hard_check(pair)
    if max(pair) < software_limit and abs(pair[0] - pair[1]) <= 2:
        return []
    confirmed = []
    for _ in range(2):
        pause(0.02)
        fresh = read_pair()
        hard_check(fresh)
        confirmed.append(list(fresh))
        if max(fresh) >= software_limit:
            raise TemperatureLimitError("High temperature persisted on confirmation")
        if abs(fresh[0] - fresh[1]) > 2:
            raise TemperatureDataError(
                "Temperature inconsistency persisted on confirmation"
            )
    return confirmed
