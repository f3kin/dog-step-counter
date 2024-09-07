import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

def set_build_flags(env):
    # Retrieve environment variables
    wifi_ssid = os.getenv('PLATFORMIO_MY_WIFI_SSID')
    wifi_password = os.getenv('PLATFORMIO_MY_WIFI_PASSWORD')
    server_url = os.getenv('PLATFORMIO_MY_SERVER_URL')
    id_token = os.getenv('PLATFORMIO_MY_ID_TOKEN')

    # Dynamically add build flags
    if wifi_ssid and wifi_password and server_url and id_token:
        env.Append(
            BUILD_FLAGS=[
                f'-D PLATFORMIO_MY_WIFI_SSID="{wifi_ssid}"',
                f'-D PLATFORMIO_MY_WIFI_PASSWORD="{wifi_password}"',
                f'-D PLATFORMIO_MY_SERVER_URL="{server_url}"',
                f'-D PLATFORMIO_MY_ID_TOKEN="{id_token}"'
            ]
        )
    else:
        print("Error: Missing one or more environment variables.")

# This is the main script that PlatformIO runs
def after_upload(source, target, env):
    set_build_flags(env)