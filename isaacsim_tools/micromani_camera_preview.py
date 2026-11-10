"""Latest-frame HTTP preview, loopback only. Never queues frames or writes training data."""
import base64
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

class CameraPreview:
    def __init__(self, port, max_fps=None):
        self.payload=None;self.sequence=0;self.last_attempt=0.;self.last_stamps=None
        self.interval=0. if max_fps is None else 1/max_fps
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path!='/frame':self.send_error(404);return
                payload=owner.payload
                self.send_response(200 if payload else 503)
                self.send_header('Content-Type','application/json');self.send_header('Cache-Control','no-store')
                body=payload or b'{"status":"waiting for cameras"}'
                self.send_header('Content-Length',str(len(body)));self.end_headers()
                try:self.wfile.write(body)
                except (BrokenPipeError,ConnectionResetError):pass
            def log_message(self,*args):pass
        self.server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
        self.server.daemon_threads=True
        threading.Thread(target=self.server.serve_forever,daemon=True).start()

    def publish(self, cameras, sim_time):
        now=time.monotonic()
        if now-self.last_attempt<self.interval:return
        self.last_attempt=now
        from PIL import Image
        from micromani_record_session import camera_sample
        samples={name:camera_sample(camera) for name,camera in cameras.items()}
        stamps={name:stamp for name,(_,stamp) in samples.items()}
        if stamps==self.last_stamps:return
        images={}
        for name,(rgb,stamp) in samples.items():
            buf=io.BytesIO();Image.fromarray(rgb).save(buf,format='JPEG',quality=90)
            images[name]={'jpeg':base64.b64encode(buf.getvalue()).decode('ascii'),'render_time':stamp,'width':640,'height':480,'sample_rate_hz':getattr(cameras[name],'get_frequency',lambda:None)()}
        self.sequence+=1
        payload={'sequence':self.sequence,'published_at':time.time(),'simulation_time':float(sim_time),'cameras':images}
        self.payload=json.dumps(payload,separators=(',',':')).encode()
        self.last_stamps=stamps

    def close(self):
        self.server.shutdown();self.server.server_close()
