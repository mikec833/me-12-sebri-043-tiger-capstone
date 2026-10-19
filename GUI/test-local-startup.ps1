param(
    [string]$Browser = 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe'
)
$ErrorActionPreference = 'Stop'
$page = ([System.Uri](Join-Path $PSScriptRoot 'controller_v4_6.html')).AbsoluteUri
$profile = Join-Path ([System.IO.Path]::GetTempPath()) ('tigerball-startup-' + [guid]::NewGuid())
# Use normal browser file-origin restrictions; do not pass --allow-file-access-from-files.
$dom = (& $Browser --headless --disable-gpu --no-first-run "--user-data-dir=$profile" --dump-dom --virtual-time-budget=3000 $page 2>$null | Out-String)
if (-not $dom) { throw 'Browser failed to run' }
if ($dom -notmatch 'id="status"[^>]*>Disconnected</div>') { throw 'Controller failed to initialize from a local file' }
if ($dom -notmatch '<button id="connect" class="connect">Connect</button>') { throw 'Connect is disabled' }
if ($dom -notmatch '<button id="arm"[^>]*disabled') { throw 'Arm must remain disabled before connecting' }
if ($dom -notmatch '<g id="compassTicks"><line') { throw 'Pi compass failed to initialize' }
Write-Output 'PASS: local-file startup, Connect available, Arm disabled, Pi compass initialized'
