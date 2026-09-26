import re, sys, serial
port=sys.argv[1]
with serial.Serial(port,115200,timeout=1) as s:
    while True:
        line=s.readline().decode(errors='replace').strip()
        if line.startswith('P'):
            print(line)
