import board
import busio
import digitalio
import pwmio
import rotaryio
from time import sleep, monotonic
import adafruit_midi
from adafruit_midi.system_exclusive import SystemExclusive
from adafruit_midi.control_change import ControlChange

# --- LCD Settings ---
i2c = busio.I2C(board.GP15, board.GP14)
LCD_addr = 0x27
LCD_EN, LCD_BL = 0x04, 0x08
CMD, CHR = 0x00, 0x01
LINE1, LINE2 = 0x80, 0xC0
buf = bytearray(2)

while not i2c.try_lock():
    pass

def LCD_write(bits, mode):
    data = (bits & 0xF0) | mode
    buf[0] = data | LCD_EN | LCD_BL
    buf[1] = data | LCD_BL
    i2c.writeto(LCD_addr, buf)
    sleep(0.0001)
    data = ((bits << 4) & 0xF0) | mode
    buf[0] = data | LCD_EN | LCD_BL
    buf[1] = data | LCD_BL
    i2c.writeto(LCD_addr, buf)
    sleep(0.0001)

def LCD_init():
    for cmd in [0x33, 0x32, 0x06, 0x0C, 0x28, 0x01]:
        LCD_write(cmd, CMD)
    sleep(0.002)

def LCD_clear():
    LCD_write(0x01, CMD)
    sleep(0.002)

def LCD_cursor(x, y):
    LCD_write((LINE1 if y == 0 else LINE2) + x, CMD)

def LCD_print(message):
    for c in message:
        LCD_write(ord(c), CHR)

# --- Parameter Management ---
# Set it to empty initially so it can be populated dynamically from JUCE
Param_Config = {}
cc_keys = []
current_idx = 0

def display_param_and_value(name, val_text):
    """Adjust the value display position based on the name length"""
    LCD_clear()
    if len(name) <= 16:
        LCD_cursor(0, 0)
        LCD_print(name)
        LCD_cursor(0, 1)
        LCD_print(str(val_text))
    else:
        LCD_cursor(0, 0)
        LCD_print(name[:16])
        LCD_cursor(0, 1)
        remaining_name = name[16:]
        LCD_print(f"{remaining_name} {val_text}")

# --- LED and PWM Settings ---
led_tact1 = pwmio.PWMOut(board.GP22, frequency=5000, duty_cycle=0)
led_val = pwmio.PWMOut(board.GP26, frequency=5000, duty_cycle=0)
led_tact2 = pwmio.PWMOut(board.GP17, frequency=5000, duty_cycle=0)

# --- Input/Output Pin Settings ---
encoder = rotaryio.IncrementalEncoder(board.GP27, board.GP28)
last_encoder_pos = encoder.position

def setup_sw(pin):
    io = digitalio.DigitalInOut(pin)
    io.direction = digitalio.Direction.INPUT
    io.pull = digitalio.Pull.UP
    return io

sw1Up, sw1Down = setup_sw(board.GP5), setup_sw(board.GP7)
sw2Up, sw2Down = setup_sw(board.GP20), setup_sw(board.GP8)

# --- UART MIDI Settings (GP0 = TX -> DSP Pico GP1, GP1 = RX <- DSP Pico GP0) ---
uart = busio.UART(board.GP0, board.GP1, baudrate=115200, timeout=0.001, receiver_buffer_size=512)
midi_in = adafruit_midi.MIDI(midi_in=uart, in_buf_size=256)
midi_out = adafruit_midi.MIDI(midi_out=uart, out_channel=0)

LCD_init()
LCD_print("Waiting for DSP")

# Request initial parameter configuration (CONF) from DSP Pico via CC 127
midi_out.send(ControlChange(127, 127))
last_sync_req_time = monotonic()

last_states = [True, True, True, True] 

while True:
    # --- 1. MIDI Reception (Feedback from DSP Pico via UART SysEx) ---
    msg = midi_in.receive()
    while msg is not None:
        if isinstance(msg, SystemExclusive) and msg.manufacturer_id == b'\x7d':
            try:
                val_str = "".join([chr(b) for b in msg.data])
                parts = val_str.split(",")
                
                print(f"Received SysEx: {parts}") 
                
                # Initial parameter configuration sent from DSP Pico
                if parts[0] == "CONF":
                    # CONF,cc,id,val_str,max,midi_val
                    cc = int(parts[1])
                    p_id = parts[2]
                    p_val = parts[3]
                    p_max = int(parts[4])
                    p_midi = int(parts[5])
                    
                    Param_Config[cc] = {
                        "name": p_id, 
                        "val_str": p_val,   # Save the values for display
                        "cur_midi": p_midi, # Save current MIDI values
                        "max_val": p_max  
                    }
                    
                    old_len = len(cc_keys)
                    cc_keys = sorted(Param_Config.keys())
                    
                    # Update the display when the first parameter is received
                    if old_len == 0 and len(cc_keys) > 0:
                        config = Param_Config[cc_keys[current_idx]]
                        display_param_and_value(config["name"], config["val_str"])
                        
                # Parameter value update sent from DSP Pico
                elif parts[0] == "UPDT":
                    # UPDT,id,val_str,midi_val
                    p_id = parts[1]
                    p_val = parts[2]
                    p_midi = int(parts[3])
                    
                    for cc, config in Param_Config.items():
                        if config["name"] == p_id:
                            config["val_str"] = p_val
                            config["cur_midi"] = p_midi
                            # If the parameter currently selected in sw1 is updated, the display is also updated
                            if cc_keys and cc == cc_keys[current_idx]:
                                print(f"Updating display for {p_id}: {p_val}", current_idx)
                                display_param_and_value(config["name"], config["val_str"])
                            break
            except Exception as e:
                print(f"SysEx Error: {e}")
        msg = midi_in.receive()

    # Periodically request CONF if parameters have not yet been received (e.g. if DSP Pico booted first)
    if not cc_keys:
        now = monotonic()
        if now - last_sync_req_time >= 0.5:
            midi_out.send(ControlChange(127, 127))
            last_sync_req_time = now
        sleep(0.01)
        continue

    # --- 2. Parameter Switching (sw1) + LED Control ---
    if not sw1Up.value or not sw1Down.value:
        led_tact1.duty_cycle = 65535
    else:
        led_tact1.duty_cycle = 0

    if not sw1Up.value and last_states[0]:
        current_idx = (current_idx + 1) % len(cc_keys)
        config = Param_Config[cc_keys[current_idx]]
        display_param_and_value(config["name"], config["val_str"])
    
    if not sw1Down.value and last_states[1]:
        current_idx = (current_idx - 1) % len(cc_keys)
        config = Param_Config[cc_keys[current_idx]]
        display_param_and_value(config["name"], config["val_str"])

    # --- 3. Encoder Operation + LED Dimming ---
    current_pos = encoder.position
    delta = current_pos - last_encoder_pos
    
    if delta != 0:
        config = Param_Config[cc_keys[current_idx]]
        # Change per click (if you want to change by 127 over 20 clicks)
        step_val = 127 / (20 * 3) 
        new_midi = config["cur_midi"] + (delta * step_val)
        config["cur_midi"] = max(0, min(127, int(new_midi)))
        
        midi_out.send(ControlChange(cc_keys[current_idx], config["cur_midi"]))
        last_encoder_pos = current_pos

    # Dimmer the LED to match the current MIDI value
    led_val.duty_cycle = int(Param_Config[cc_keys[current_idx]]["cur_midi"] * 516)

    # --- 4. Parameter Operation (sw2) + LED Control ---
    if not sw2Up.value or not sw2Down.value:
        led_tact2.duty_cycle = 65535
    else:
        led_tact2.duty_cycle = 0

    if not sw2Up.value and last_states[2]:
        config = Param_Config[cc_keys[current_idx]]
        # Step calculation based on resolution
        step = max(1, 128 // config["max_val"]) if config["max_val"] > 0 else 1
        config["cur_midi"] = min(127, config["cur_midi"] + step)
        midi_out.send(ControlChange(cc_keys[current_idx], config["cur_midi"]))
    
    if not sw2Down.value and last_states[3]:
        config = Param_Config[cc_keys[current_idx]]
        step = max(1, 128 // config["max_val"]) if config["max_val"] > 0 else 1
        config["cur_midi"] = max(0, config["cur_midi"] - step)
        midi_out.send(ControlChange(cc_keys[current_idx], config["cur_midi"]))

    # State Preservation and Weight
    last_states = [sw1Up.value, sw1Down.value, sw2Up.value, sw2Down.value]
    sleep(0.01)
