"""Compare measured FIR output with its prediction; do not label predictions verified."""
import argparse,csv,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def report(folder):
    root=Path(folder);meta=json.loads((root/'design.json').read_text())
    rows=list(csv.DictReader((root/'validation.csv').open()))
    f=np.array([float(r['hz']) for r in rows]);h=np.array([float(r['h1']) for r in rows])
    h2=np.array([float(r['h2']) for r in rows]);h3=np.array([float(r['h3']) for r in rows])
    lo,hi=meta['band_hz'];band=(f>=lo)&(f<=hi)
    valid=band&(h2>=0)&(h3>=0)
    harmonic=np.hypot(h2[valid],h3[valid])/h[valid]
    stats=dict(measured_min_mg=float(h[band].min()),measured_max_mg=float(h[band].max()),
        measured_ripple_db=float(20*np.log10(h[band].max()/h[band].min())),
        harmonic_ratio_median=float(np.median(harmonic)),harmonic_ratio_max=float(max(harmonic)),
        vsys_min=min(float(r['vsys']) for r in rows),points=len(rows),
        scope='Single unit-amplitude sine sweep; LF/HF sustained path. LFi silent. Not proof for arbitrary waveforms or other grip/drive levels.')
    (root/'validation.json').write_text(json.dumps(stats,indent=2))
    z=np.load(root/'filter.npz')
    fig,ax=plt.subplots(2,1,figsize=(10,7),sharex=True)
    ax[0].semilogx(z['f'],z['predicted_mg'],'--',label='Model prediction')
    ax[0].semilogx(f,h,'o-',label='Measured enclosure output')
    ax[0].set_ylabel('Fundamental acceleration (mg peak)');ax[0].legend()
    ax[1].semilogx(f[valid],harmonic*100,'o-');ax[1].set_ylabel('H2 + H3 ratio (%)');ax[1].set_xlabel('Frequency (Hz)')
    for a in ax:a.grid(True,alpha=.3);a.axvspan(lo,hi,alpha=.08,color='green')
    fig.suptitle('LF/HF equalization; LFi reserved for asymmetric impacts')
    fig.tight_layout();fig.savefig(root/'measured-response.png',dpi=150)
    print(json.dumps(stats,indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('folder');a=p.parse_args();report(a.folder)
