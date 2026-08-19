$ErrorActionPreference = 'Stop'

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$elfPath = Join-Path $repositoryRoot 'Debug\ra8p1_project1.elf'
$nmCommand = Get-Command arm-none-eabi-nm -ErrorAction SilentlyContinue

if ($null -ne $nmCommand)
{
    $nm = $nmCommand.Source
}
else
{
    $nm = Get-ChildItem `
        -Path 'C:\Renesas\RA\e2studio_*\toolchains\gcc_arm\*\bin\arm-none-eabi-nm.exe' `
        -File | Sort-Object LastWriteTime -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}

if (-not (Test-Path -LiteralPath $elfPath))
{
    throw 'SAFE ELF is missing. Run a clean default build first.'
}

if ([string]::IsNullOrWhiteSpace($nm))
{
    throw 'arm-none-eabi-nm was not found.'
}

$symbols = (& $nm -C $elfPath) -join "`n"

if ($LASTEXITCODE -ne 0)
{
    throw 'Failed to read ELF symbols.'
}

$forbiddenSymbols = @(
    'actuator_manager_apply_us',
    'flight_control_update',
    'flight_control_prop_load_vibration_update',
    'quad_x_mixer_apply'
)

$requiredSymbols = @(
    'motor_output_all_stop',
    'HardFault_Handler',
    'MemManage_Handler',
    'BusFault_Handler',
    'UsageFault_Handler',
    'vApplicationStackOverflowHook'
)

foreach ($symbol in $forbiddenSymbols)
{
    if ($symbols -match "(?m)\b$([regex]::Escape($symbol))$")
    {
        throw "SAFE ELF unexpectedly contains '$symbol'."
    }
}

foreach ($symbol in $requiredSymbols)
{
    if ($symbols -notmatch "(?m)\b$([regex]::Escape($symbol))$")
    {
        throw "SAFE ELF is missing required safety symbol '$symbol'."
    }
}

Write-Output 'SAFE ELF symbol boundary passed.'
