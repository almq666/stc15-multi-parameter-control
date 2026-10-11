$ErrorActionPreference = 'Stop'
$toolRoot = 'C:\Keil_v5\C51'
$projectRoot = $PSScriptRoot
$objects = Join-Path $projectRoot 'User\Objects'
New-Item -ItemType Directory -Force $objects | Out-Null
$env:C51INC = "$toolRoot\INC;$toolRoot\INC\STC"
$env:C51LIB = "$toolRoot\LIB"
$sources = @('User\main.c','Driver\Init.c','Driver\Key.c','Driver\Seg.c','Driver\Led.c','Driver\iic.c','Driver\onewire.c','Driver\ds1302.c','Driver\ultrasound.c')
$objectFiles = @()
foreach ($source in $sources) {
    $name = [IO.Path]::GetFileNameWithoutExtension($source)
    $object = Join-Path $objects ($name + '.obj')
    & "$toolRoot\BIN\C51.exe" (Join-Path $projectRoot $source) "INCDIR($projectRoot\Driver,$toolRoot\INC,$toolRoot\INC\STC)" "OBJECT($object)" 'SMALL' 'OPTIMIZE(8,SPEED)'
    if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $source" }
    $objectFiles += $object
}
& "$toolRoot\BIN\A51.exe" "$toolRoot\LIB\STARTUP.A51" "OBJECT($objects\STARTUP.obj)"
if ($LASTEXITCODE -ne 0) { throw 'Startup assembly failed' }
$objectFiles += "$objects\STARTUP.obj"
& "$toolRoot\BIN\BL51.exe" ($objectFiles -join ',') 'TO' "$objects\Xifeng" 'RAMSIZE(256)' 'XDATA(0X0000-0X06FF)' 'CODE(0X0000-0XEFF8)'
if ($LASTEXITCODE -gt 1) { throw 'Link failed' }
& "$toolRoot\BIN\OH51.exe" "$objects\Xifeng"
if ($LASTEXITCODE -ne 0) { throw 'HEX generation failed' }
