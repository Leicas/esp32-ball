"""Full-drive solo signature capture; B firmware command uses positive-only LFi."""
import csv,json,time,datetime
from pathlib import Path
import serial
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parent/'sweeps'/('mounted-signature-'+datetime.datetime.now().strftime('%Y%m%d-%H%M%S'))
root.mkdir(parents=True)
print('OUTPUT',root,flush=True)
rows=[];raw=[];complete=False
with serial.Serial('COM6',115200,timeout=.3) as s, (root/'serial.log').open('w') as log:
 time.sleep(1);s.reset_input_buffer()
 s.write(b'-'*21);time.sleep(.5);s.write(b'+'*20);time.sleep(.5)
 s.reset_input_buffer();s.write(b'B')
 deadline=time.monotonic()+300
 while time.monotonic()<deadline:
  line=s.readline().decode(errors='replace').strip()
  if not line:continue
  log.write(line+'\n');log.flush()
  if line.startswith('$sw,'):
   a=line.split(',');row=[a[1]]+list(map(float,a[2:]));rows.append(row)
   print('CHANNEL',a[1],'HZ',a[2],'RMS_mg',a[7],'VSYS',a[8],flush=True)
  elif line.startswith('$raw,'):
   a=line.split(',');raw.append([a[1],float(a[2]),*map(int,a[3:])])
  elif line.startswith('# sweep end'):complete=True;break
  elif 'skipped' in line or line.startswith('rst:'):raise RuntimeError(line)
 s.write(b'x')
with (root/'response.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['channel','hz','drive_fs','h1_mg','h2_mg','h3_mg','rms_mg','vsys','n']);w.writerows(rows)
with (root/'acceleration.csv').open('w',newline='') as f:
 w=csv.writer(f);w.writerow(['channel','hz','t_us','x_lsb','y_lsb','z_lsb']);w.writerows(raw)
assert complete,'Incomplete capture; partial data saved'
assert len(rows)==123,f'Expected 123 response rows, got {len(rows)}'
names={'1':'LFi (positive half-wave)','3':'HF (sine)','4':'LF (sine)'}
fig,axes=plt.subplots(2,1,figsize=(11,8),sharex=True)
summary={'range_hz':[20,500],'drive_peak_fs':1,'accel_mg_per_lsb':.488,'accel_range_g':16,'complete':complete,'raw_samples':len(raw),'channels':{}}
for ch,name in names.items():
 r=np.array([v[1:] for v in rows if v[0]==ch and v[1]>0]);a=np.array([v[2:] for v in raw if v[0]==ch])
 assert np.all(r[:,1]==1),'Not full drive'
 axes[0].loglog(r[:,0],r[:,2],label=name)
 axes[1].semilogx(r[:,0],r[:,5],label=name)
 peak=int(np.argmax(r[:,2]));clip=int(np.sum(np.abs(a[:,1:])>=32760))
 summary['channels'][name]={'peak_hz':r[peak,0],'peak_fundamental_mg':r[peak,2],'max_rms_mg':float(max(r[:,5])),'min_vsys':float(min(r[:,6])),'clipped_axis_samples':clip,'raw_samples':len(a),'min_samples_per_tone':float(min(r[:,7]))}
axes[0].set_ylabel('Fundamental acceleration (mg peak)');axes[1].set_ylabel('Broadband acceleration (mg RMS)');axes[1].set_xlabel('Drive frequency (Hz)')
for ax in axes:ax.grid(True,which='both',alpha=.25);ax.legend()
fig.suptitle('Mounted enclosure — full-drive individual actuator signatures\nLFi half-wave harmonics are part of its drive, not a distortion measurement')
fig.tight_layout();fig.savefig(root/'signature.png',dpi=160)
(root/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(summary,indent=2),flush=True)
