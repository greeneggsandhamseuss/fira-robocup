"""Very basic hardware test for DroneCommunication.

For a two-radio test, run this file on both computers after giving each copy
a different NAME. Keep TEAM identical on both computers.
"""

import time

from drone_communication import DroneCommunication


NAME = "drone_a"       # Use a different name on each drone.
TEAM = "rescue1"       # Must be the same on every drone.
PORT = None            # Example: "/dev/ttyUSB0"; None = auto-detect.


def main():
    radio = DroneCommunication(name=NAME, team=TEAM, port=PORT)

    try:
        discarded = radio.flush()
        print(f"Connected as {NAME}. Cleared {discarded} old message(s).")

        while True:
            try:
                radio.send(f"hello from {NAME}")
                print("Test message sent.")
            except RuntimeError as error:
                print(f"Could not send yet: {error}")
                print("Start another radio on the same team, then run this again.")

            time.sleep(1.05)
    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        radio.close()


if __name__ == "__main__":
    main()
