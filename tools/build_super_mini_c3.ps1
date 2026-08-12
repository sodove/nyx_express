param(
    [switch]$Flash,
    [string]$Port
)

$ErrorActionPreference = "Stop"

$repo = (Resolve-Path (Join-Path $PSScriptRoot ".."))
$buildDir = Join-Path $repo "build_super_mini_c3"

if (-not $env:IDF_PATH) {
    $siblingIdf = Join-Path $repo "..\nyxdash\esp-idf-v5.5.4"
    if (Test-Path -LiteralPath (Join-Path $siblingIdf "tools\idf.py")) {
        $env:IDF_PATH = (Resolve-Path $siblingIdf).Path
    } else {
        throw "IDF_PATH is not set and ..\nyxdash\esp-idf-v5.5.4 was not found."
    }
}

$python310 = "C:\Users\sodovaya\AppData\Local\Programs\Python\Python310"
if (Test-Path -LiteralPath (Join-Path $python310 "python.exe")) {
    $env:Path = "$python310;$python310\Scripts;$env:Path"
}

$exportPs1 = Join-Path $env:IDF_PATH "export.ps1"
if (Test-Path -LiteralPath $exportPs1) {
    . $exportPs1
}

$idfPy = Join-Path $env:IDF_PATH "tools\idf.py"
if (-not (Test-Path -LiteralPath $idfPy)) {
    throw "Cannot find $idfPy"
}

$idfPython = (Get-Command python -ErrorAction Stop).Source
if ($env:IDF_PYTHON_ENV_PATH) {
    $venvPython = Join-Path $env:IDF_PYTHON_ENV_PATH "python.exe"
    if (Test-Path -LiteralPath $venvPython) {
        $idfPython = $venvPython
    }
}

function Invoke-Idf([string[]]$Arguments) {
    & $idfPython $idfPy @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "idf.py failed with exit code $LASTEXITCODE"
    }
}

Push-Location $repo
try {
Invoke-Idf @("-B", $buildDir, "-DIDF_TARGET=esp32c3", "-DHW_NAME=NyxExpress", "reconfigure")
    Invoke-Idf @("-B", $buildDir, "build")

    if ($Flash) {
        $flashArgs = @("-B", $buildDir, "flash")
        if ($Port) {
            $flashArgs = @("-B", $buildDir, "-p", $Port, "flash")
        }
        Invoke-Idf $flashArgs
    }
}
finally {
    Pop-Location
}
