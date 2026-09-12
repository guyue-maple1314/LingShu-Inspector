# Run colcon tests and pytest tests.
param(
  [string]$Workspace = (Join-Path $PSScriptRoot "..\ros2_ws"),
  [switch]$SkipColcon
)

$ErrorActionPreference = "Continue"

if (-not $SkipColcon) {
  if (Get-Command colcon -ErrorAction SilentlyContinue) {
    Push-Location $Workspace
    try {
      & colcon test --event-handlers console_direct+
      & colcon test-result --verbose
    }
    finally {
      Pop-Location
    }
  }
  else {
    Write-Warning "colcon not found; skipping colcon test."
  }
}

if (Get-Command python -ErrorAction SilentlyContinue) {
  $tests = Join-Path $Workspace "src\inspection_tests"
  & python -m pytest $tests -q
}
else {
  Write-Warning "python not found; skipping pytest tests."
}
