"""Very basic hardware test for DroneCommunication.

For a two-radio test, run this file on both computers after giving each copy
a different NAME. Keep TEAM identical on both computers.
"""

import time

from drone_communication import DroneCommunication


NAME = "drone_b"       # Use a different name on each drone.
TEAM = "rescue1"       # Must be the same on every drone.
PORT = None            # Example: "/dev/ttyUSB0"; None = auto-detect.


def main():
    radio = DroneCommunication(name=NAME, team=TEAM, port=PORT)

    try:
        discarded = radio.flush()
        print(f"Connected as {NAME}. Cleared {discarded} old message(s).")


        while True:
            message = radio.receive()
            if message is not None:
                print(f"Received from {message['sender']}: {message['text']}")
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        radio.close()


if __name__ == "__main__":
    main()
