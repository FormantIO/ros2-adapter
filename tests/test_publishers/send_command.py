import datetime
import os

from formant.sdk.cloud.v1 import Client as FormantClient


# Put your device ID here
MY_DEVICE_ID = ""
# Put your organization ID here
MY_ORG_ID = ""


def get_current_isodate():
    return datetime.datetime.now(tz=datetime.timezone.utc).isoformat()


if __name__ == "__main__":
    fclient = FormantClient()
    
    print("Sending command...")
    command_result = fclient.create_command(
        {
            "deviceId": MY_DEVICE_ID,
            "organizationId": MY_ORG_ID,
            "command": "my.command.for.publisher",
            "parameter": {"scrubberTime": get_current_isodate()},
        }
    )
    command_id = command_result["id"]
    print("Command ID:\n%s" % command_id)
