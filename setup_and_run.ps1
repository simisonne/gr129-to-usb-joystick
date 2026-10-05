# One-shot setup + run for the GR129 -> Liftoff bridge on Windows.
# Run from PowerShell:
#   powershell -ExecutionPolicy Bypass -File .\setup_and_run.ps1
#
# Installs (if missing): Python, ViGEmBus driver, the vgamepad package,
# then starts the feeder.

$ErrorActionPreference = "Continue"
$dir = Split-Path -Parent $MyInvocation.MyCommand.Path

Write-Host "=== 1/4 Python ==="
$py = (Get-Command py -ErrorAction SilentlyContinue)
if (-not $py) { $py = (Get-Command python -ErrorAction SilentlyContinue) }
if (-not $py) {
    Write-Host "No Python found - installing via winget (needs admin)..."
    winget install --id Python.Python.3.12 -e --accept-source-agreements --accept-package-agreements
    $py = (Get-Command py -ErrorAction SilentlyContinue)
    if (-not $py) { $py = (Get-Command python -ErrorAction SilentlyContinue) }
}
if (-not $py) { Write-Host "Python still missing - open a NEW PowerShell and re-run."; exit 1 }
$pycmd = $py.Source
if ($pycmd -like "*py.exe") { $pycmd = "py" }
Write-Host "using: $pycmd"

Write-Host "=== 2/4 ViGEmBus driver (virtual Xbox pad) ==="
$bus = Get-Package -Name "*ViGEm*" -ErrorAction SilentlyContinue
if (-not $bus) {
    winget install --id Nefarius.ViGEmBus -e --accept-source-agreements --accept-package-agreements
    if (-not $?) { Write-Host "winget failed - install manually: https://github.com/nefarius/ViGEmBus/releases" }
}
Write-Host "(a reboot may be needed once if the driver was just installed)"

Write-Host "=== 3/4 vgamepad ==="
& $pycmd -m pip install --quiet vgamepad pyserial
if (-not $?) { Write-Host "pip failed - try manually: pip install vgamepad"; exit 1 }

Write-Host "=== 3b/4 inbound UDP 5005 rule ==="
$rule = Get-NetFirewallRule -DisplayName "GR129 feeder UDP5005" -ErrorAction SilentlyContinue
if (-not $rule) {
    try {
        New-NetFirewallRule -DisplayName "GR129 feeder UDP5005" -Direction Inbound -Protocol UDP -LocalPort 5005 -Action Allow -Profile Private -ErrorAction Stop | Out-Null
        Write-Host "firewall rule added"
    } catch {
        Write-Host "could not add firewall rule (need admin) - allow the firewall prompt instead"
    }
}

Write-Host "=== 4/4 feeder ==="
Write-Host "Windows Firewall may ask for permission on first run - allow it (Private network)."
Write-Host "Move the GR129 sticks: you should see pkt/s and values change."
& $pycmd (Join-Path $dir "feeder.py") @args
