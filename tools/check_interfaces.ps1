# Verify interface files exist and topic names are consistent across layers.
param(
  [string]$Root = (Join-Path $PSScriptRoot "..")
)

$ErrorActionPreference = "Continue"
$fail = $false

$interfaces = Join-Path $Root "ros2_ws\src\inspection_interfaces"
$messages = @(
  "RobotState", "OperatorInstruction", "InspectionAlert", "StructuredGoal",
  "Task", "EnergyConstraint", "TaskDecision", "TerrainObservation",
  "CorridorState", "FusionPose", "SemanticAlarm", "ThermalMeasurement",
  "AcousticDiagnosis"
)
$services = @("ValidateGoal", "ResumeTask")
$actions = @("ExecuteTask", "NavigateGoal")

foreach ($m in $messages) {
  if (-not (Test-Path (Join-Path $interfaces "msg\$m.msg"))) {
    Write-Warning "Missing interface: msg/$m.msg"
    $fail = $true
  }
}
foreach ($s in $services) {
  if (-not (Test-Path (Join-Path $interfaces "srv\$s.srv"))) {
    Write-Warning "Missing interface: srv/$s.srv"
    $fail = $true
  }
}
foreach ($a in $actions) {
  if (-not (Test-Path (Join-Path $interfaces "action\$a.action"))) {
    Write-Warning "Missing interface: action/$a.action"
    $fail = $true
  }
}

$cpp = Get-Content (Join-Path $Root "ros2_ws\src\inspection_execution_cpp\include\inspection_execution_cpp\common\topic_names.hpp") -Raw
$py = Get-Content (Join-Path $Root "ros2_ws\src\inspection_planning_py\inspection_planning_py\common\topic_names.py") -Raw
$js = Get-Content (Join-Path $Root "ros2_ws\src\inspection_hmi_js\src\ros\topics.js") -Raw

$topicPattern = '"/[a-z_]+"'
$cppTopics = @([regex]::Matches($cpp, $topicPattern) | ForEach-Object { $_.Value.Trim('"') } | Sort-Object -Unique)
$pyTopics = @([regex]::Matches($py, $topicPattern) | ForEach-Object { $_.Value.Trim('"') } | Sort-Object -Unique)
$jsTopics = @([regex]::Matches($js, $topicPattern) | ForEach-Object { $_.Value.Trim('"') } | Sort-Object -Unique)

foreach ($t in $cppTopics) {
  if (($pyTopics -notcontains $t) -and ($jsTopics -notcontains $t)) {
    Write-Warning "Topic $t is inconsistent across the three layers."
  }
}

if ($fail) {
  Write-Error "Interface check failed."
  exit 1
}

Write-Host "Interface check passed: $($messages.Count) msg / $($services.Count) srv / $($actions.Count) action."
