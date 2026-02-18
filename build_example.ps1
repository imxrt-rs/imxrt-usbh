# build_example.ps1 - Build an example and produce a HEX file, showing only errors
# Usage: .\build_example.ps1 -Example <name> [-HexFile <output.hex>]
#
# Default hex file name is <example>.hex in the current directory.

param(
    [Parameter(Mandatory=$true)]
    [string]$Example,

    [Parameter(Mandatory=$false)]
    [string]$HexFile = "$Example.hex"
)

$Target = "thumbv7em-none-eabihf"
$ElfPath = "target/$Target/release/examples/$Example"

Write-Host "Building example '$Example' -> '$HexFile' ..." -ForegroundColor Cyan

# cargo build writes plain status lines to stdout and diagnostic messages (errors,
# warnings, notes) to stderr.  Redirect stderr to stdout so we can filter it.
$buildOutput = & cargo build --release --target $Target --example $Example 2>&1

# Filter: keep lines that indicate an error (but not "warning: ..." or "note: ...")
# Patterns kept:
#   error[Exxxx]: ...
#   error: ...
#   ^  --> src/...  (source location lines following an error)
#   ^  |            (code-context lines following an error)
#   ^  = ...        (help/label lines following an error)
# We use a simple approach: collect all lines that are part of an error block.

$inError = $false
$errorLines = @()

foreach ($line in $buildOutput) {
    $text = if ($line -is [System.Management.Automation.ErrorRecord]) {
        $line.Exception.Message
    } else {
        "$line"
    }

    if ($text -match '^error(\[E\d+\])?:') {
        $inError = $true
    } elseif ($text -match '^warning(\[.+\])?:' -or $text -match '^(\s*Compiling|\s*Finished|\s*Blocking|\s*Fresh)') {
        $inError = $false
    }

    if ($inError) {
        $errorLines += $text
    }
}

if ($errorLines.Count -gt 0) {
    Write-Host ""
    Write-Host "BUILD ERRORS:" -ForegroundColor Red
    $errorLines | ForEach-Object { Write-Host $_ -ForegroundColor Red }
    Write-Host ""
    Write-Host "Build FAILED for example '$Example'." -ForegroundColor Red
    exit 1
}

# Check exit code from cargo
if ($LASTEXITCODE -ne 0) {
    Write-Host ""
    Write-Host "Build FAILED for example '$Example' (exit code $LASTEXITCODE)." -ForegroundColor Red
    Write-Host "(No error lines captured; run 'cargo build --release --target $Target --example $Example' manually for full output.)" -ForegroundColor Yellow
    exit $LASTEXITCODE
}

Write-Host "Build succeeded. Converting ELF to HEX ..." -ForegroundColor Cyan

$objcopyOutput = & rust-objcopy -O ihex $ElfPath $HexFile 2>&1
if ($LASTEXITCODE -ne 0) {
    Write-Host "rust-objcopy failed:" -ForegroundColor Red
    $objcopyOutput | ForEach-Object { Write-Host $_ -ForegroundColor Red }
    exit $LASTEXITCODE
}

Write-Host "Done: $HexFile" -ForegroundColor Green
