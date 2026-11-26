# Builds the modified Teensy firmwares (raw GNSS for TDCP, receiver commands, firmware update over Ethernet) and
# puts the .hex files into this repository's Firmwares/AgOpenGPS-OTA-RawGNSS folder, from where the AgIO module
# configurator downloads them once they are committed and pushed.
#   .\build.ps1
#
# Needs arduino-cli with the Teensy package 1.57.3 (these firmwares do not build with 1.58 or later):
#   arduino-cli config add board_manager.additional_urls https://www.pjrc.com/teensy/package_teensy_index.json
#   arduino-cli core install teensy:avr@1.57.3

param([string]$ArduinoCli = "$env:USERPROFILE\.local\arduino-cli\arduino-cli.exe")

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$out = Join-Path $here '..\Firmwares\AgOpenGPS-OTA-RawGNSS'
$build = Join-Path $env:TEMP 'aog-teensy-build'

# sketch folder, output (folder/name), and for a variant: the source folder and the text changes that make it
$firmwares = @(
    @{ Sketch = 'AIO_v4_Firmware';           Out = 'All-In-One\AIO v4 (OTA & RawGNSS)' }
    @{ Sketch = 'Autosteer_gps_teensy_v2_5'; Out = 'All-In-One\AIO v2.5 (OTA & RawGNSS)' }
    @{ Sketch = 'AIO_v4_CANBUS_Firmware';    Out = 'CANBUS\CANBUS All-In-One (OTA & RawGNSS)' }
    @{ Sketch = 'AIO_v4_CANBUS_CommonRail';  Out = 'CANBUS\CANBUS CommonRail (OTA & RawGNSS)'; From = 'AIO_v4_CANBUS_Firmware'
       Changes = @(
           @('AIO_v4_CANBUS_Firmware.ino', '#define isAllInOneBoard', '//#define isAllInOneBoard   // Tony''s (CommonRail) board'),
           @('zGnssBridge.ino', 'CANBUS All-In-One (OTA & RawGNSS)', 'CANBUS CommonRail (OTA & RawGNSS)')) }
    @{ Sketch = 'Keya_chriskinal';           Out = 'Keya\Keya (OTA & RawGNSS)' }
)

foreach ($f in $firmwares) {
    $sketchDir = Join-Path $here $f.Sketch
    if ($f.From) {
        # a variant: a copy of its source with a few lines changed
        $sketchDir = Join-Path $build ('src\' + $f.Sketch)
        if (Test-Path $sketchDir) { Remove-Item -Recurse -Force $sketchDir }
        New-Item -ItemType Directory -Force $sketchDir | Out-Null
        Copy-Item (Join-Path $here ($f.From + '\*')) $sketchDir -Recurse
        Rename-Item (Join-Path $sketchDir ($f.From + '.ino')) ($f.Sketch + '.ino')
        foreach ($c in $f.Changes) {
            $file = Join-Path $sketchDir ($c[0] -replace [regex]::Escape($f.From + '.ino'), ($f.Sketch + '.ino'))
            $text = [IO.File]::ReadAllText($file)
            if (-not $text.Contains($c[1])) { throw "$($f.Sketch): '$($c[1])' not found in $($c[0])" }
            [IO.File]::WriteAllText($file, $text.Replace($c[1], $c[2]))
        }
    }
    # the shared parts, the same in every sketch
    Copy-Item (Join-Path $here 'common\AogGnssBridge.h') $sketchDir -Force
    Copy-Item (Join-Path $here 'common\AogNetUpdate.h') $sketchDir -Force

    $dir = Join-Path $build $f.Sketch
    try {
        & $ArduinoCli compile --fqbn teensy:avr:teensy41 --output-dir $dir $sketchDir
        $built = $LASTEXITCODE -eq 0
    }
    finally {
        # the copies go again: the shared parts live only in common/
        if (-not $f.From) { Remove-Item (Join-Path $sketchDir 'AogGnssBridge.h'), (Join-Path $sketchDir 'AogNetUpdate.h') -ErrorAction SilentlyContinue }
    }
    if (-not $built) { throw "$($f.Sketch) did not build" }
    $target = Join-Path $out ($f.Out + '.hex')
    New-Item -ItemType Directory -Force (Split-Path -Parent $target) | Out-Null
    Copy-Item (Join-Path $dir "$($f.Sketch).ino.hex") $target -Force
    Write-Host "$($f.Sketch) -> $($f.Out).hex"
}
Write-Host "Written to $out - commit and push for the AgIO module configurator to download them"
