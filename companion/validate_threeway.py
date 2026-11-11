"""Capture the actual threeway FIR path via bringup command E, always save rows."""
import argparse
import csv
import time
from pathlib import Path
import serial

def capture(port,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with serial.Serial(port,115200,timeout=.3) as s, path.open('w',newline='') as file:
        writer=csv.writer(file)
        writer.writerow(['mode','hz','amp','h1','h2','h3','rms','vsys','n'])
        time.sleep(1);s.reset_input_buffer();s.write(b'E')
        started=False;count=0
        try:
            end=time.monotonic()+100
            while time.monotonic()<end:
                line=s.readline().decode(errors='replace').strip()
                if line.startswith('# sweep begin'): started=True;print(line,flush=True)
                if line.startswith('$sw,E,'):
                    writer.writerow(line[4:].split(','));file.flush();count+=1
                    print(line,flush=True)
                if line.startswith('# sweep end') and started:
                    if count!=40: raise RuntimeError(f'Incomplete validation: {count}/40 points')
                    return
                if 'skipped' in line: raise RuntimeError(line)
            raise TimeoutError('No completed filtered sweep')
        finally:
            s.write(b'x')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('port');p.add_argument('csv')
    a=p.parse_args();capture(a.port,a.csv)
