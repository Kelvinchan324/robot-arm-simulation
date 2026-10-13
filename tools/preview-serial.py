"""Local-only browser fixture. Never included in dist or GitHub Pages.

Run: python tools/preview-serial.py
Open: http://127.0.0.1:43818/serial-test.html
"""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT/'dist'), **kwargs)

    def do_GET(self):
        path = self.path.split('?')[0]
        if path == '/serial-test.html':
            html = (ROOT/'dist/index.html').read_text(encoding='utf-8')
            html = html.replace('<script type="module" src="app.js', '<script type="module" src="/__test__/browser-serial-fixture.js"></script><script type="module" src="app.js')
            data = html.encode('utf-8'); mime = 'text/html; charset=utf-8'
        elif path in ('/__test__/browser-serial-fixture.js', '/__test__/serial-calibration.test.json'):
            file = ROOT/'tools'/path.rsplit('/', 1)[-1]
            data = file.read_bytes(); mime = 'text/javascript' if file.suffix == '.js' else 'application/json'
        else:
            return super().do_GET()
        self.send_response(200); self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)


if __name__ == '__main__':
    print('FAKE SERIAL ONLY: http://127.0.0.1:43818/serial-test.html', flush=True)
    ThreadingHTTPServer(('127.0.0.1', 43818), Handler).serve_forever()
