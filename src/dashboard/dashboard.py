# app.py

import threading
import json
import time
from datetime import datetime

import serial
from flask import Flask
import dash
from dash import dcc, html, Output, Input

# ---- CONFIGURE YOUR SERIAL PORT HERE ----
SERIAL_PORT = 'COM6'  # Update to your actual Windows COM port
BAUD_RATE = 115200
# -----------------------------------------

# Shared data structures
sensor_data = {}   # e.g. { "MPRLS": {"pressure": 1001.3, "ts": datetime(...)}, ... }
logs = []          # list of {"ts": datetime, "type": "...", "source": "...", "message": "..."}

# Start the Flask server
server = Flask(__name__)
app = dash.Dash(__name__, server=server)

def serial_reader():
    """Background thread: read lines from serial, parse JSON, update sensor_data & logs."""
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    except serial.SerialException as e:
        print(f"Could not open serial port: {e}")
        return

    while True:
        try:
            line = ser.readline().decode('utf-8').strip()
            if not line:
                continue

            payload = json.loads(line)
            # Expecting a list of message objects
            for entry in payload:
                t = entry.get('type')
                content = entry.get('content', {})
                source = content.get('source', 'UNKNOWN')

                if t == 'SENSOR_DATA':
                    # store the entire sensorData dict plus timestamp
                    sd = content.get('sensorData', {})
                    sensor_data[source] = {
                        'values': sd,
                        'ts': datetime.utcnow()
                    }

                elif t in ('INFO','WARNING','ERROR'):
                    msg = content.get('message', '')
                    logs.append({
                        'ts': datetime.utcnow(),
                        'type': t,
                        'source': source,
                        'message': msg
                    })

            # trim logs to last 200 messages
            if len(logs) > 200:
                del logs[:-200]

        except json.JSONDecodeError:
            # skip invalid JSON
            continue
        except Exception as e:
            print(f"Error in serial_reader: {e}")
            time.sleep(1)

# Start background serial thread
threading.Thread(target=serial_reader, daemon=True).start()

# Dash layout
app.layout = html.Div(style={'display': 'flex', 'height': '100vh', 'fontFamily': 'Arial'}, children=[

    # Left: Logs
    html.Div(style={
        'width': '35%', 'padding': '10px', 'borderRight': '1px solid #ccc', 'overflowY': 'scroll'
    }, children=[
        html.H3("System Logs"),
        html.Div(id='log-box', style={'whiteSpace': 'pre-wrap', 'fontFamily': 'Courier New'})
    ]),

    # Right: Sensor Data
    html.Div(style={'flex': '1', 'padding': '10px'}, children=[
        html.H3("Sensor Readings"),
        html.Div(id='sensor-panel')
    ]),

    # Interval to trigger updates every second
    dcc.Interval(id='interval-update', interval=1000, n_intervals=0)
])

@app.callback(
    Output('log-box', 'children'),
    Input('interval-update', 'n_intervals')
)
def update_logs(n):
    """Render the last logs in color-coded fashion."""
    lines = []
    for entry in logs[-50:]:  # show last 50
        ts = entry['ts'].strftime('%H:%M:%S')
        typ = entry['type']
        src = entry['source']
        msg = entry['message']
        # color by type
        color = {
            'INFO': 'black',
            'WARNING': 'orange',
            'ERROR': 'red'
        }.get(typ, 'black')
        lines.append(
            html.Div(f"{ts} [{src}] {typ}: {msg}", style={'color': color})
        )
    return lines

@app.callback(
    Output('sensor-panel', 'children'),
    Input('interval-update', 'n_intervals')
)
def update_sensors(n):
    """Render each sensor’s latest readings and time since update."""
    now = datetime.utcnow()
    panels = []
    for src, data in sensor_data.items():
        vals = data['values']
        ts = data['ts']
        elapsed = (now - ts).total_seconds()
        # build a table of key/value
        rows = []
        for k, v in vals.items():
            rows.append(html.Tr([html.Td(k), html.Td(f"{v}")]))
        panels.append(html.Div(style={
            'border': '1px solid #888', 'borderRadius': '5px',
            'padding': '8px', 'marginBottom': '10px'
        }, children=[
            html.H4(src),
            html.P(f"Last update: {elapsed:.1f}s ago", style={'fontStyle':'italic', 'fontSize':'90%'}),
            html.Table(rows, style={'width':'100%'})
        ]))
    return panels

if __name__ == '__main__':
    app.run(debug=True)
