param([string]$Port = 'COM10', [int]$HttpPort = 9150)
$ErrorActionPreference = 'Stop'
& "$PSScriptRoot/.venv/Scripts/python.exe" "$PSScriptRoot/receiver_ui.py" --port $Port --http-port $HttpPort
