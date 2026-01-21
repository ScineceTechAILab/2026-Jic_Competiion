
import requests
import sys

try:
    # Use localhost since we are on the same machine
    r = requests.get('http://127.0.0.1:8000/api/camera/stream', stream=True, timeout=2)
    print(f"Status Code: {r.status_code}")
    print(f"Headers: {r.headers}")
    # Read a bit of content
    for chunk in r.iter_content(chunk_size=1024):
        print(f"Received chunk of size {len(chunk)}")
        break
except Exception as e:
    print(f"Error: {e}")
