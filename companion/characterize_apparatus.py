"""Four-actuator characterization; fixed Mad Catz microphone at 10cm.
Records per-tone raw accelerometer and microphone WAV, plus scalar measurements.
"""
import argparse,csv,json,time,threading
from pathlib import Path
import numpy as np
import serial,sounddevice as sd
from scipy.io import wavfile
P=argparse.ArgumentParser();P.add_argument('--out',required=True);P.add_argument('--phase',choices=['pilot','dense','levels','high','pairs','quick','quickpairs','negative','lfi-shapes'],default='pilot');args=P.parse_args()
root=Path(args.out);root.mkdir(parents=True,exist_ok=True);(root/'raw').mkdir(exist_ok=True)
SR=48000;device=next(i for i,d in enumerate(sd.query_devices()) if 'Mad Catz' in d['name'] and d['hostapi']==2)
sd.check_input_settings(device=device,channels=1,samplerate=SR,dtype='float32')
chunks=[];errors=[];lock=threading.Lock()
def callback(indata,frames,info,status):
 with lock:
  chunks.append((time.perf_counter()+info.inputBufferAdcTime-info.currentTime,indata[:,0].copy()))
  if status:errors.append(str(status))
stream=sd.InputStream(device=device,channels=1,samplerate=SR,dtype='float32',blocksize=480,latency='low',callback=callback)
s=serial.Serial('COM6',115200,timeout=.5);time.sleep(1);s.reset_input_buffer();s.write(b'x')
fields=['id','phase','mask','hz','amp','invert','duration','n','sample_hz','imu_rms_mg','imu_h1_mg','imu_h2_mg','imu_h3_mg','imu_clip','audio_rms_dbfs','audio_h1_dbfs','audio_h2_dbfs','audio_h3_dbfs','audio_thd','audio_clip','vsys','audio_status']
path=root/'measurements.csv';new=not path.exists();fh=path.open('a',newline='');writer=csv.DictWriter(fh,fieldnames=fields)
if new:writer.writeheader()
existing={r['id'] for r in csv.DictReader(path.open())} if not new else set()
def rms(x):return np.sqrt(np.mean(x*x))
def db(x):return float(20*np.log10(max(float(x),1e-12)))
def harmonics(t,x,f,maxhz):
 if f<=0:return [0.,0.,0.]
 x=x-np.mean(x,axis=0);out=[]
 for h in (1,2,3):
  if h*f>maxhz:out.append(float('nan'));continue
  w=np.column_stack([np.cos(2*np.pi*f*h*t),np.sin(2*np.pi*f*h*t),np.ones(len(t))])
  fit=np.linalg.lstsq(w,x,rcond=None)[0][:2];out.append(float(np.sqrt(np.sum(fit*fit))))
 return out
def measure(mask,hz,amp,inv=0):
 ident=f'{args.phase}_{mask}_{hz:g}_{amp:g}_{inv}'
 if ident in existing:return
 dur=max(.30,4/hz) if 0<hz<=1000 else .20
 dur=min(4.,dur) if hz else .5
 if args.phase=='quick':dur=min(.5,max(.12,2/hz)) if hz else .3
 with lock:chunks.clear();errors.clear()
 s.write(f'Q{mask} {hz:.5f} {amp} {dur:.6f} {inv}\n'.encode())
 deadline=time.monotonic()+dur+8;begin=None;raw=None
 while time.monotonic()<deadline:
  line=s.readline().decode(errors='replace').strip()
  if line.startswith('$begin,'):begin=time.perf_counter()
  if line.startswith('$data,'):
   _,n,vs=line.split(',');n=int(n);raw=s.read(n*10)
   if len(raw)!=n*10:raise RuntimeError('short binary capture')
  if line=='$end':break
  if line.startswith('$err') or 'rst:' in line:raise RuntimeError(line)
 else:raise RuntimeError('tone timeout')
 if begin is None or raw is None:raise RuntimeError('missing tone markers')
 time.sleep(.12)
 with lock:blocks=list(chunks);status=list(errors)
 audio=np.concatenate([b for _,b in blocks]);end=blocks[-1][0]+len(blocks[-1][1])/SR
 # Host timing aligns steady-state magnitudes, NOT absolute cross-sensor phase.
 times=np.concatenate([bt+np.arange(len(b))/SR for bt,b in blocks])
 keep=(times>=begin+.025)&(times<=begin+dur-.025)
 y=audio[keep]
 if len(y)<100:raise RuntimeError('insufficient microphone capture')
 arr=np.frombuffer(raw,dtype=[('t','<u4'),('xyz','<i2',(3,))]);t=arr['t'].astype(float)*1e-6;x=arr['xyz'].astype(float)*.488
 np.savez_compressed(root/'raw'/(ident+'.npz'),t_us=arr['t'],xyz=arr['xyz'],audio_start=begin-(end-len(audio)/SR),audio_duration=dur)
 wavfile.write(root/'raw'/(ident+'.wav'),SR,audio)
 h=harmonics(t,x,hz,1400);ah=harmonics(np.arange(len(y))/SR,y,hz,21000)
 row=dict(id=ident,phase=args.phase,mask=mask,hz=hz,amp=amp,invert=inv,duration=dur,n=n,sample_hz=(n-1)/(t[-1]-t[0]),imu_rms_mg=float(np.sqrt(np.sum(np.var(x,axis=0)))),imu_h1_mg=h[0],imu_h2_mg=h[1],imu_h3_mg=h[2],imu_clip=int(np.sum(abs(arr['xyz'].astype(int))>=32760)),audio_rms_dbfs=db(rms(y)),audio_h1_dbfs=db(ah[0]),audio_h2_dbfs=db(ah[1]),audio_h3_dbfs=db(ah[2]),audio_thd=float(np.sqrt(ah[1]**2+ah[2]**2)/max(ah[0],1e-12)),audio_clip=int(np.sum(abs(y)>=.999)),vsys=float(vs),audio_status=';'.join(status))
 writer.writerow(row);fh.flush()
 if hz%50==0 or args.phase!='dense':print(ident,'imu',round(row['imu_rms_mg']), 'audio',round(row['audio_h1_dbfs'],1),'clip',row['imu_clip'],row['audio_clip'],'fs',round(row['sample_hz']),flush=True)
 if row['audio_clip']>0 or row['imu_clip']>0:raise RuntimeError('clipping detected; adjust before continuing')
 if status:raise RuntimeError('audio capture overflow')
try:
 with stream:
  time.sleep(.3)
  if args.phase=='pilot':
   measure(0,0,0)
   for mask in (1,2,4,8):
    for f in (40,100,160,300,800,1000,5000,15000,20000):measure(mask,f,.25)
  elif args.phase=='quick':
   measure(0,0,0)
   for mask in (1,2,4,8):
    for f in sorted(set(np.round(np.geomspace(5,1000,32)).astype(int))):measure(mask,int(f),1.)
    for f in np.geomspace(1200,20000,12):measure(mask,round(float(f),2),1.)
  elif args.phase=='dense':
   for mask in (1,2,4,8):
    measure(0,0,0)
    for f in range(1,1001):measure(mask,f,1.)
  elif args.phase=='levels':
   for mask in (1,2,4,8):
    for f in sorted(set(np.round(np.geomspace(10,1000,55)).astype(int))):
     for amp in (.25,.5):measure(mask,int(f),amp)
  elif args.phase=='high':
   for mask in (1,2,4,8):
    for f in np.geomspace(1050,20000,110):measure(mask,round(float(f),2),1.)
  elif args.phase=='lfi-shapes':
   measure(0,0,0)
   for amp in (.06,.12,.25,.5):
    for shape in (16,32,33):
     for f in (25,40,55,65,85,120):measure(1,f,amp,shape)
  elif args.phase=='negative':
   for amp in (.25,.5,1.):
    for f in (20,35,45,55,65,80,100,130):measure(1,f,amp,1)
  elif args.phase=='quickpairs':
   for mask,inv,fs in ((10,8,(80,130)),(6,4,(160,300,500))):
    for f in fs:
     for polarity in (0,inv):measure(mask,f,.5,polarity)
  elif args.phase=='pairs':
   for mask,inv in ((6,4),(10,8),(12,8)):
    for f in (40,60,80,100,120,160,200,250,320,400,500,650,800,1000):
     for polarity in (0,inv):measure(mask,f,.5,polarity)
finally:
 s.write(b'x');s.close();fh.close()
print('COMPLETE',args.phase,flush=True)
