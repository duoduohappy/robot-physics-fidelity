param([ValidateSet('apollo','g1')][string]$Asset='apollo', [string]$Keyframe)
$ErrorActionPreference='Stop'
$workspaceRoot=(Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
$previousPythonPath=$env:PYTHONPATH
$previousGl=$env:MUJOCO_GL
$previousCache=$env:WARP_CACHE_PATH
if (-not $Keyframe) { $Keyframe=if ($Asset -eq 'apollo') { 'stand' } else { 'home' } }
try {
    $env:PYTHONPATH=Join-Path $workspaceRoot 'work/usd-pilot-deps'
    $env:MUJOCO_GL='disable'
    $env:WARP_CACHE_PATH=Join-Path $workspaceRoot 'work/warp-cache'
    python (Join-Path $PSScriptRoot 'validate.py') --asset $Asset --keyframe $Keyframe --explicit-joint-limits
    $commandExit=$LASTEXITCODE
} finally {
    $env:PYTHONPATH=$previousPythonPath
    $env:MUJOCO_GL=$previousGl
    $env:WARP_CACHE_PATH=$previousCache
}
exit $commandExit
