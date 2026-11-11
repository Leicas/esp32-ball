import json,os,socket,sqlite3,threading,time,csv,io
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler,ThreadingHTTPServer
DB=os.environ.get('DB','/data/battery.sqlite3')
@contextmanager
def connect():
 c=sqlite3.connect(DB,timeout=10)
 c.execute('PRAGMA journal_mode=WAL')
 c.execute('CREATE TABLE IF NOT EXISTS readings (received REAL, device TEXT, boot TEXT, seq INTEGER, payload TEXT, UNIQUE(device,boot,seq))')
 try:
  yield c
  c.commit()
 finally:c.close()
def accept(data):
 p=json.loads(data)
 if not isinstance(p,dict) or p.get('kind')!='a7_battery' or p.get('v')!=1:raise ValueError()
 for key in ['device','boot']:
  if not isinstance(p.get(key),str) or not 1<=len(p[key])<=64:raise ValueError()
 for key,low,high in [('seq',0,4294967295),('uptime_s',0,4294967295),('vsys_mv',0,6000),('usb',0,1)]:
  if type(p.get(key))!=int or not low<=p[key]<=high:raise ValueError()
 with connect() as c:c.execute('INSERT OR IGNORE INTO readings VALUES (?,?,?,?,?)',(time.time(),p['device'],p['boot'],p['seq'],json.dumps(p)))
def receive():
 s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);s.bind(('0.0.0.0',9876))
 while True:
  data,_=s.recvfrom(4096)
  try:accept(data)
  except (ValueError,UnicodeError,TypeError):pass
class Handler(BaseHTTPRequestHandler):
 def do_GET(self):
  if self.path=='/health':body=b'ok';kind='text/plain'
  elif self.path=='/api/export.csv':
   with connect() as c:rows=c.execute('SELECT received,payload FROM readings ORDER BY received').fetchall()
   out=io.StringIO();w=csv.writer(out);keys=['device','boot','seq','uptime_s','vsys_mv','usb','rssi','reset_reason','mode','gain','audio_drops','battery_pct','remaining_s','estimate_model'];w.writerow(['received_unix']+keys)
   for t,p in rows:
    p=json.loads(p);w.writerow([t]+[p.get(k,'') for k in keys])
   body=out.getvalue().encode();kind='text/csv'
  elif self.path=='/api/readings':
   with connect() as c:rows=c.execute('SELECT received,payload FROM readings ORDER BY received DESC LIMIT 10000').fetchall()
   body=json.dumps([dict(json.loads(p),received=t,age_s=max(0,time.time()-t)) for t,p in rows]).encode();kind='application/json'
  elif self.path=='/':
   body=open('/app/index.html','rb').read();kind='text/html'
  else:self.send_error(404);return
  self.send_response(200);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(body)));self.end_headers();self.wfile.write(body)
if __name__=='__main__':
 with connect():pass
 threading.Thread(target=receive,daemon=True).start();ThreadingHTTPServer(('0.0.0.0',8080),Handler).serve_forever()
