from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


DIST = Path(__file__).resolve().parent / "dist"
server = ThreadingHTTPServer(
    ("127.0.0.1", 8000),
    partial(SimpleHTTPRequestHandler, directory=str(DIST)),
)

print("Serving robot arm simulation at http://localhost:8000 (Ctrl+C to stop)", flush=True)
server.serve_forever()
