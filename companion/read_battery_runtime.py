"""Read the saved battery duration without starting or overwriting a test."""
import argparse,json,time
from pathlib import Path
import serial
p=argparse.ArgumentParser();p.add_argument('port',default='COM6',nargs='?');args=p.parse_args()
with serial.Serial(args.port,115200,timeout=.3) as s:
 time.sleep(1);s.reset_input_buffer();s.write(b'Q')
 deadline=time.monotonic()+5
 while time.monotonic()<deadline:
  line=s.readline().decode(errors='replace').strip()
  if line.startswith('# battery_test='):
   fields=dict(x.split('=',1) for x in line[2:].split() if '=' in x)
   seconds=int(fields['elapsed_s']);print(line)
   print(f"Recorded duration: {seconds//3600}h {(seconds//60)%60}m {seconds%60}s")
   if fields['battery_test']=='interrupted':print('Last saved checkpoint: lower bound, not proof of battery exhaustion.')
   target=Path(__file__).parent/'sweeps'/('battery-runtime-'+time.strftime('%Y%m%d-%H%M%S')+'.json')
   target.parent.mkdir(exist_ok=True);target.write_text(json.dumps(fields,indent=2));print('Saved:',target)
   break
 else:raise SystemExit('No battery result received; check firmware and USB connection.')
