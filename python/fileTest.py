import os
import json


os.chdir("real_time_data_base")
telemetry_db = open("telemetry.json", "w")

toWrite = {
    "telemetry": {
        "time": 0,
        "altitude": 0
    }
}

json.dump(toWrite, telemetry_db, indent=4)
telemetry_db.close()