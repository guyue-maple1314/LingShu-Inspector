# Build the ROS 2 workspace with colcon.
param(
  [string]$Workspace = (Join-Path $PSScriptRoot "..\ros2_ws"),
  [switch]$SymlinkInstall
)

$ErrorActionPreference = "Stop"

if (-not (Get-Command colcon -ErrorAction SilentlyContinue)) {
  Write-Error "colcon not found. Please source the ROS 2 environment first."
  exit 1
}

Push-Location $Workspace
try {
  $buildArgs = @("build", "--cmake-args", "-DCMAKE_BUILD_TYPE=Release")
  if ($SymlinkInstall) {
    $buildArgs += "--symlink-install"
  }
  & colcon @buildArgs
  exit $LASTEXITCODE
}
finally {
  Pop-Location
}
