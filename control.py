import sys
from pynput import keyboard
import serial

ser  = serial.Serial('COM7', 115200, timeout=1)

keys_state = {}
motion_keys_stack = [] 

level = 2

def on_press(key):
    global level
    
    if key == keyboard.Key.esc:
        return False

    try:
        key_char = key.char
    except AttributeError:
        return  

    if key_char in ['a', 'w', 's', 'd']:
        if key_char not in motion_keys_stack:

            if motion_keys_stack:
                print(f"end\n".encode())
                ser.write(f"end\n".encode())

            ser.write(f"{key_char} start\n".encode())
            print(f"{key_char} start\n".encode())
            motion_keys_stack.append(key_char)
    
    if key_char == 'i':
        if level <= 3:
            level = level +1

        print("accelerating, level: " +str(level))
        ser.write(f"i\n".encode())
    if key_char == 'k':
        if level >= 1:
            level = level -1

        print("slowing down, level: " +str(level))
        ser.write(f"k\n".encode())

def on_release(key):
    try:
        key_char = key.char
    except AttributeError:
        return 

    if key_char in motion_keys_stack:
        
        if key_char == motion_keys_stack[-1]:
            print(f"end\n".encode())
            ser.write(f"end\n".encode())

        if key_char == motion_keys_stack[-1]:
            motion_keys_stack.remove(key_char)
            if motion_keys_stack:
                resumed_key = motion_keys_stack[-1]
                ser.write(f"{resumed_key} start\n".encode())
                print(f"{resumed_key} start\n".encode())
        else:
            motion_keys_stack.remove(key_char)

level = 2
with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
    listener.join()

ser.close()
print("keyboard Listener and serial stopped")
