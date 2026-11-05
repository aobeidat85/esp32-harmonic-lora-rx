param([double]$Seconds = 10)
$ErrorActionPreference = 'Stop'
Push-Location $PSScriptRoot
try {
    $pythonPath = Join-Path $PSScriptRoot '.venv/Scripts/python.exe'
    $capturePath = 'captures/heltec-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.cu8'
    & $pythonPath capture.py --seconds $Seconds --output $capturePath
    if ($LASTEXITCODE -ne 0) { throw 'SDR capture failed' }
    & $pythonPath decode_capture.py $capturePath
    if ($LASTEXITCODE -ne 0) { throw 'LoRa decoding failed' }
} finally { Pop-Location }
