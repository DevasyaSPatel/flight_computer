import time
import threading
import json
import serial
import serial.tools.list_ports
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
socketio = SocketIO(app, cors_allowed_origins="*")

# Global variables
serial_port = None
is_running = True
serial_thread = None
current_port_name = None

def find_arduino_port():
    """
    Auto-detects a likely Arduino/Serial device.
    Returns the port name (e.g., 'COM3') or None.
    """
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        # Common descriptions for Arduino/ESP devices
        if "Arduino" in p.description or "CH340" in p.description or "CP210" in p.description or "USB Serial" in p.description:
            return p.device
    # Fallback: just return the first available port if any
    if len(ports) > 0:
        return ports[0].device
    return None

def read_serial_data():
    """
    Reads data from the serial port and emits it to the client.
    Format: packetCount,state,alt,maxAlt,vel,ax,ay,az,gx,gy,gz,temp
    """
    global serial_port, current_port_name
    
    while is_running:
        try:
            if serial_port is None or not serial_port.is_open:
                port_name = find_arduino_port()
                if port_name:
                    print(f"Attempting to connect to {port_name}...")
                    try:
                        serial_port = serial.Serial(port_name, 115200, timeout=1)
                        current_port_name = port_name
                        time.sleep(2) # Wait for connection to settle
                        socketio.emit('serial_status', {'connected': True, 'port': port_name})
                        print(f"Connected to {port_name}")
                    except Exception as e:
                        print(f"Failed to connect: {e}")
                        time.sleep(2)
                else:
                    # No port found
                    socketio.emit('serial_status', {'connected': False, 'message': 'No device found'})
                    time.sleep(2)
                continue

            if serial_port.in_waiting > 0:
                try:
                    line = serial_port.readline().decode('utf-8').strip()
                    if not line:
                        continue
                        
                    # Parse CSV
                    # Expected: count,state,alt,maxAlt,vel,ax,ay,az,gx,gy,gz,temp
                    parts = line.split(',')
                    if len(parts) >= 12:
                        # Map State ID to Name
                        state_map = {
                            '0': 'IDLE',
                            '1': 'ASCENT',
                            '2': 'APOGEE',
                            '3': 'DESCENT',
                            '4': 'LANDED'
                        }
                        
                        state_id = parts[1]
                        state_name = state_map.get(state_id, "UNKNOWN")

                        # Parse Orientation (Gyro is rate, but for viz we need absolute)
                        # Since we don't have absolute orientation (quaternion) from FC yet,
                        # we will integrate gyro or just map accel to tilt for simple viz.
                        # For this specific request, we'll map Accel to Pitch/Roll (simplified)
                        ax = float(parts[5])
                        ay = float(parts[6])
                        az = float(parts[7])
                        
                        # Simple tilt calculation (in radians)
                        # This is a basic approximation for visualization
                        import math
                        pitch = math.atan2(ay, math.sqrt(ax*ax + az*az))
                        roll = math.atan2(-ax, math.sqrt(ay*ay + az*az))
                        yaw = 0 # No magnetometer, so yaw drift is expected or ignored

                        data = {
                            "time": float(parts[0]) / 10.0, # Assuming ~10Hz packet rate, or just use count
                            "packet_count": int(parts[0]),
                            "state": state_name,
                            "altitude": float(parts[2]),
                            "max_altitude": float(parts[3]),
                            "velocity": float(parts[4]),
                            "max_velocity": 0.0, # FC doesn't send max vel, we can track it client side or here
                            "acceleration": math.sqrt(ax*ax + ay*ay + az*az), # Total magnitude
                            "max_acceleration": 0.0, # Track client side
                            "temperature": float(parts[11]),
                            "orientation": {
                                "pitch": pitch,
                                "yaw": yaw,
                                "roll": roll
                            },
                            "raw": {
                                "ax": ax, "ay": ay, "az": az,
                                "gx": float(parts[8]), "gy": float(parts[9]), "gz": float(parts[10])
                            }
                        }
                        
                        socketio.emit('telemetry_data', data)
                        
                except ValueError:
                    pass # Malformed line
                except Exception as e:
                    print(f"Error reading line: {e}")
            
            else:
                time.sleep(0.01)

        except Exception as e:
            print(f"Serial Loop Error: {e}")
            if serial_port:
                serial_port.close()
            serial_port = None
            socketio.emit('serial_status', {'connected': False})
            time.sleep(2)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def client_connect():
    print('Client connected')
    # Send current status immediately
    if serial_port and serial_port.is_open:
        emit('serial_status', {'connected': True, 'port': current_port_name})
    else:
        emit('serial_status', {'connected': False})

if __name__ == '__main__':
    # Start serial thread
    serial_thread = threading.Thread(target=read_serial_data)
    serial_thread.daemon = True
    serial_thread.start()
    
    print("Starting Flask Server...")
    socketio.run(app, debug=True, host='0.0.0.0', port=5000)

