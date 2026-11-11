"""Generate measured response figures from the four-actuator capture."""
import csv,json,argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
p=argparse.ArgumentParser();p.add_argument('root');a=p.parse_args();root=Path(a.root)
rows=list(csv.DictReader((root/'measurements.csv').open()))
for r in rows:
 for k in r:
  if k not in ('id','phase','audio_status'):r[k]=float(r[k])
names={1:'LFi (one-sided pulses)',2:'MF',4:'HF',8:'LF'}
colors={1:'#a855f7',2:'#ef9b20',4:'#3b82f6',8:'#16a085'}
fig,axs=plt.subplots(3,1,figsize=(13,13),constrained_layout=True)
summary={}
for mask,name in names.items():
 d=sorted([r for r in rows if r['phase'] in ('dense','quick') and r['hz']<=1000 and r['mask']==mask],key=lambda r:r['hz'])
 h=sorted([r for r in rows if r['phase'] in ('high','quick') and r['hz']>1000 and r['mask']==mask],key=lambda r:r['hz'])
 if not d:continue
 f=np.array([r['hz'] for r in d]);v=np.array([r['imu_h1_mg'] for r in d]);rms=np.array([r['imu_rms_mg'] for r in d])
 axs[0].semilogx(f,v,label=name,color=colors[mask]);axs[1].semilogx(f,rms,label=name,color=colors[mask])
 ah=d+h;axs[2].semilogx([r['hz'] for r in ah],[r['audio_h1_dbfs'] for r in ah],label=name,color=colors[mask],alpha=.85)
 peak=d[int(np.argmax(v))];summary[name]={'points_1_1000':len(d),'high_points':len(h),'peak_hz':peak['hz'],'peak_mg':peak['imu_h1_mg'],'max_rms_mg':float(max(rms)),'min_vsys':min(r['vsys'] for r in d),'imu_clip':sum(r['imu_clip'] for r in d),'audio_clip':sum(r['audio_clip'] for r in d)}
for ax in axs:ax.grid(True,which='both',alpha=.2);ax.legend();ax.set_xlabel('Drive frequency / pulse repetition (Hz)')
axs[0].set_ylabel('Acceleration fundamental, peak vector (mg)');axs[0].set_title('Full-scale single-actuator response — onboard IMU')
axs[1].set_ylabel('Total acceleration, RMS vector (mg)');axs[1].set_title('Includes harmonics and intentional LFi impact energy')
axs[2].set_ylabel('Microphone fundamental (dBFS peak)');axs[2].set_title('Mad Catz microphone, 10 cm — relative sound only, uncalibrated')
fig.savefig(root/'frequency-response.png',dpi=170);plt.close(fig)
fig,axs=plt.subplots(2,1,figsize=(12,9),constrained_layout=True)
for mask,name in names.items():
 for amp,style in ((.25,':'),(.5,'--'),(1.,'-')):
  d=sorted([r for r in rows if r['mask']==mask and r['amp']==amp and r['phase'] in ('levels','dense','quick')],key=lambda r:r['hz'])
  if not d:continue
  axs[0].semilogx([r['hz'] for r in d],[r['imu_h1_mg']/amp for r in d],style,color=colors[mask],label=f'{name} {amp:g} FS')
  if mask!=1:axs[1].semilogx([r['hz'] for r in d],[100*np.sqrt(r['imu_h2_mg']**2+r['imu_h3_mg']**2)/max(r['imu_h1_mg'],1e-9) for r in d],style,color=colors[mask])
axs[0].set_title('Level comparison: divergence indicates compression or changing mechanics');axs[0].set_ylabel('Fundamental / drive amplitude (mg)');axs[0].legend(ncol=3,fontsize=8)
axs[1].set_title('IMU second + third harmonic ratio (only where harmonics < 1.4 kHz)');axs[1].set_ylabel('Harmonic ratio (%)');axs[1].set_ylim(0,100)
for ax in axs:ax.set_xlabel('Hz');ax.grid(True,which='both',alpha=.2)
fig.savefig(root/'levels-and-harmonics.png',dpi=170);plt.close(fig)
(root/'summary.json').write_text(json.dumps(summary,indent=2));print(json.dumps(summary,indent=2))
