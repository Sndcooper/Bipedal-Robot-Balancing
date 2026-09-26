import sys, tkinter as tk
import serial

port = sys.argv[1] if len(sys.argv)>1 else "COM13"
ser = serial.Serial(port,115200,timeout=.1)
root=tk.Tk(); root.title("AX-12 safe test")
idv=tk.StringVar(value="6"); pos=tk.IntVar(value=512)
def send(s): ser.write((s+'\n').encode())
tk.Entry(root,textvariable=idv,width=5).grid(row=0,column=0)
tk.Scale(root,from_=0,to=1023,variable=pos,orient="horizontal").grid(row=0,column=1)
tk.Button(root,text="Torque ON",command=lambda:send(f"TQ {idv.get()} 1")).grid(row=1,column=0)
tk.Button(root,text="Torque OFF",command=lambda:send(f"TQ {idv.get()} 0")).grid(row=1,column=1)
tk.Button(root,text="Send position",command=lambda:send(f"P {idv.get()} {pos.get()}")).grid(row=2,column=0,columnspan=2)
root.mainloop()
