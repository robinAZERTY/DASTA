import json
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os

class JSONFileHandler(FileSystemEventHandler):
    def __init__(self, json_file_path):
        self.json_file_path = json_file_path
        self.last_data = self.read_json()
        self.new_data_available = False

    def read_json(self):
        try:
            with open(self.json_file_path, 'r') as json_file:
                data = json.load(json_file)
            return data
        except (json.JSONDecodeError, FileNotFoundError):
            # Gérer les erreurs de décodage JSON ou de fichier non trouvé
            return None

    def on_modified(self, event):
        #compare the 2 paths with os.path.samefile(path1, path2)
        if os.path.samefile(event.src_path, self.json_file_path):
            new_data = self.read_json()
            if new_data is not None and new_data != self.last_data:
                self.last_data = new_data
                self.new_data_available = True

def createJSONFileHandler(jsonFile):
    event_handler = JSONFileHandler(jsonFile)
    observer = Observer()
    root = os.path.dirname(jsonFile)
    observer.schedule(event_handler, path=root, recursive=True)
    observer.start()
    return event_handler


def watch_json_file(json_file_path):
    event_handler = JSONFileHandler(json_file_path)
    observer = Observer()
    observer.schedule(event_handler, path='.', recursive=True)
    observer.start()

    try:
        print("Start watching JSON file")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stop watching JSON file")
        observer.stop()
    observer.join()

if __name__ == "__main__":
    json_file_path = "real time data base/telemetry.json"
    watch_json_file(json_file_path)