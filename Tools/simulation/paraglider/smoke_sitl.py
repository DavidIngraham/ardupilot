"""Short real-SITL JSON transport/physics smoke test, not an AUTO acceptance test."""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import tempfile
import time
from json_backend import Backend

parser=argparse.ArgumentParser()
parser.add_argument('--frames',type=int,default=800)
args=parser.parse_args()
root=Path(__file__).resolve().parents[3]
backend=Backend()
with tempfile.TemporaryDirectory(prefix='parafoil-sitl-') as directory:
    log=Path(directory)/'sitl.log'
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock, log.open('w') as output:
        sock.bind(('127.0.0.1',9002))
        sock.settimeout(20)
        process=subprocess.Popen([str(root/'build/sitl/bin/arduplane'),
            '--model','JSON:127.0.0.1','--serial0','udpclient:127.0.0.1:14550','--speedup','1','--home','45,-121,100,0',
            '--defaults',str(root/'Tools/autotest/default_params/paraglider.parm')+','+
            str(Path(__file__).with_name('sitl.parm'))],cwd=directory,stdout=output,stderr=subprocess.STDOUT)
        try:
            for i in range(args.frames):
                data,address=sock.recvfrom(2048)
                if i==args.frames//2:backend.released=True
                sock.sendto(backend.respond(data),address)
            print(json.dumps(dict(frames=args.frames,seconds=backend.time,
                released=backend.released,clipped_packets=backend.clipped_packets,
                final_position=backend.state[:3].tolist()),indent=2))
        finally:
            process.terminate()
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
            output.flush()
            print(log.read_text()[-3000:])
