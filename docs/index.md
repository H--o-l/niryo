
### CodinGame

#### Site URL

<https://www.codingame.com/training/easy>

#### Google docs

<https://docs.google.com/spreadsheets/d/1koBeSrQ3yDxVRiUkARXx9e9FwEVwaJktRdL4o3opCLo/edit?usp=sharing>

<br/>

### TP seed

```py
# -*- coding: utf-8 -*-

# 1. Launch windows powershell
# 2. Go where you saved the file, for example:
#      > cd c:\Users\gim\Desktop
# 3. Use python 2.7 (keyboard issue with python 3.7):
#      > C:\Python27\python.exe '.\file.py'

import keyboard
import requests

while True:
    key = keyboard.read_event()
    if key.event_type == 'up':
        continue

    print key.name
    print key.scan_code
```
