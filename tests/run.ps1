$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path $PSScriptRoot -Parent
python -m ziglang cc "$PSScriptRoot\test_logic.c" -o "$PSScriptRoot\test_logic.exe" -Wall -Wextra
if ($LASTEXITCODE -ne 0) { throw 'Logic test compile failed' }
& "$PSScriptRoot\test_logic.exe"
if ($LASTEXITCODE -ne 0) { throw 'Logic test failed' }
$mainSource = Get-Content -Raw -Encoding UTF8 "$projectRoot\User\main.c"
$mainSource = $mainSource -replace '\binterrupt\s+[1347]\b',''
$mainSource = $mainSource -replace 'void main\(\)','void firmware_main()'
[IO.File]::WriteAllText("$PSScriptRoot\main_host.inc",$mainSource,[Text.UTF8Encoding]::new($false))
python -m ziglang cc "$PSScriptRoot\test_application.c" "-I$PSScriptRoot\mocks" "-I$projectRoot\Driver" "-I$projectRoot\User" -o "$PSScriptRoot\test_application.exe" -Wall -Wextra
if ($LASTEXITCODE -ne 0) { throw 'Application test compile failed' }
& "$PSScriptRoot\test_application.exe"
if ($LASTEXITCODE -ne 0) { throw 'Application test failed' }
