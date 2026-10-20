param([string]$Emxx = $env:EMXX)
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$repoRoot = (Resolve-Path -LiteralPath (Join-Path $projectRoot '..')).Path
$outputDir = Join-Path $projectRoot 'web/public/wasm'
New-Item -ItemType Directory -Force -Path $outputDir | Out-Null
if (-not $Emxx) { $Emxx = Join-Path $repoRoot '.cache/emsdk/upstream/emscripten/em++.bat' }
if (-not (Test-Path -LiteralPath $Emxx)) { throw 'Pass -Emxx with the Emscripten em++.bat path.' }
$env:EM_NODE_JS = (Get-Command node -ErrorAction Stop).Source.Replace('\','/')
$emscriptenDir = Split-Path -Parent (Resolve-Path -LiteralPath $Emxx).Path
$upstreamDir = Split-Path -Parent $emscriptenDir
$llvmDir = (Join-Path $upstreamDir 'bin').Replace('\','/')
$binaryenDir = $upstreamDir.Replace('\','/')
# Use a project-local configuration, including when no emsdk activation was run.
# This also prevents Emscripten from consulting a user-profile .emscripten file.
$env:EM_CONFIG = Join-Path $repoRoot '.cache/emscripten-config.py'
@"
LLVM_ROOT = '$llvmDir'
BINARYEN_ROOT = '$binaryenDir'
NODE_JS = '$env:EM_NODE_JS'
"@ | Set-Content -LiteralPath $env:EM_CONFIG -Encoding utf8
$env:EM_CACHE = Join-Path $repoRoot '.cache/emscripten'
$env:TEMP = Join-Path $repoRoot '.cache/tmp'
$env:TMP = $env:TEMP
New-Item -ItemType Directory -Force -Path $env:TEMP | Out-Null
$lib = Join-Path $repoRoot 'App/Arm/Lib'
$sources = @((Join-Path $PSScriptRoot 'deploy_ik_api.cpp'))
foreach ($part in @('myfkine26pro','myikine26pro','smartikine26pro')) {
  $sources += Join-Path $lib "$part/$part.c"
}
foreach ($file in @('rt_nonfinite.c','rtGetInf.c','rtGetNaN.c')) { $sources += Join-Path $lib "myfkine26pro/$file" }
& $Emxx @sources '-std=c++20' '-O2' '-Wno-deprecated' `
 '-sMODULARIZE=1' '-sEXPORT_ES6=1' '-sENVIRONMENT=web,worker,node' `
 '-sFILESYSTEM=0' '-sALLOW_MEMORY_GROWTH=1' `
 '-sEXPORTED_FUNCTIONS=["_malloc","_free","_deploy_ik_solve","_deploy_fk","_deploy_fk_chain","_deploy_get_joint_limits","_deploy_model_version","_deploy_trajectory_duration","_deploy_trajectory_sample"]' `
 '-sEXPORTED_RUNTIME_METHODS=["HEAPF64"]' '-o' (Join-Path $outputDir 'deploy_ik.js')
if ($LASTEXITCODE -ne 0) { throw "Emscripten failed: $LASTEXITCODE" }
Write-Host "Six-axis WASM generated in $outputDir"
