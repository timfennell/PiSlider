#!/usr/bin/env python3
"""
pislider_wake.py  —  Tiny on-demand wake trigger for PiSlider.

Listens on port 8000 at boot (uses ~8 MB RAM, ~0% CPU).
When any browser visits, it:
  1. Returns a "starting…" splash page with a JS poller
  2. Fires  systemctl start pislider  (fire-and-forget)
  3. Shuts itself down so the full app can bind port 8000

The JS on the splash page polls  /  every 2 s.  Once the full app is
responding, the browser is automatically redirected to the real UI.

When the full app later stops, pislider.service's ExecStopPost re-arms
this wake trigger so the next browser visit starts everything again.
"""

import http.server
import subprocess
import threading
import time

PORT = 8000

_SPLASH = """\
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <title>PiSlider \u2014 Starting\u2026</title>
  <style>
    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0 }
    body {
      background: #111; color: #eee;
      font-family: system-ui, sans-serif;
      display: flex; flex-direction: column;
      align-items: center; justify-content: center;
      height: 100vh; gap: 18px;
    }
    .ring {
      width: 60px; height: 60px;
      border: 5px solid #333;
      border-top-color: #e87d26;
      border-radius: 50%;
      animation: spin .85s linear infinite;
    }
    @keyframes spin { to { transform: rotate(360deg) } }
    h1  { font-size: 1.6rem; color: #e87d26; letter-spacing: .02em }
    p   { color: #888; font-size: .95rem }
    #st { color: #555; font-size: .8rem; min-height: 1.2em }
  </style>
</head>
<body>
  <div class="ring"></div>
  <h1>PiSlider is starting\u2026</h1>
  <p>Hardware initialising &mdash; please wait.</p>
  <p id="st">Waiting for app\u2026</p>

  <script>
    var attempts = 0;
    var st = document.getElementById('st');

    function check() {
      attempts++;
      st.textContent = 'Checking\u2026 (' + attempts + ')';
      fetch('/', { cache: 'no-store' })
        .then(function (r) {
          // Full app responds with the real HTML UI (not this splash).
          // Any 2xx means it's up — reload to get the actual page.
          if (r.ok) {
            st.textContent = 'Ready \u2014 loading\u2026';
            window.location.replace('/');
          } else {
            retry();
          }
        })
        .catch(retry);   // connection refused while app is starting
    }

    function retry() { setTimeout(check, 2000); }

    // First check after 7 s — the wake server exits after ~1 s, so this
    // window ensures we only hit the real app (or "not ready yet").
    setTimeout(check, 7000);
  </script>
</body>
</html>
""".encode("utf-8")

_launch_fired = threading.Event()


class _WakeHandler(http.server.BaseHTTPRequestHandler):
    """Serve the splash page and trigger the full app on first request."""

    def do_GET(self):
        self.send_response(200)
        self.send_header("Content-Type",   "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(_SPLASH)))
        self.send_header("Cache-Control",  "no-store")
        self.end_headers()
        self.wfile.write(_SPLASH)

        if not _launch_fired.is_set():
            _launch_fired.set()
            threading.Thread(target=_launch, daemon=True).start()

    def log_message(self, *args):
        pass   # keep journalctl clean


def _launch():
    """Fire pislider.service then release port 8000."""
    time.sleep(0.5)   # let the HTTP response flush to the browser

    subprocess.Popen(
        ["systemctl", "start", "pislider"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

    time.sleep(0.5)   # brief grace period before releasing the port
    _server.shutdown()


_server = http.server.HTTPServer(("", PORT), _WakeHandler)
_server.serve_forever()
