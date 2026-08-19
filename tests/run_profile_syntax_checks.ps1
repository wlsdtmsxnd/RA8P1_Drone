$ErrorActionPreference = 'Stop'

$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..')).Path
$compilerCommand = Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue

if ($null -ne $compilerCommand)
{
    $compiler = $compilerCommand.Source
}
else
{
    $compiler = Get-ChildItem `
        -Path 'C:\Renesas\RA\e2studio_*\toolchains\gcc_arm\*\bin\arm-none-eabi-gcc.exe' `
        -File | Sort-Object LastWriteTime -Descending |
        Select-Object -First 1 -ExpandProperty FullName
}

if ([string]::IsNullOrWhiteSpace($compiler))
{
    throw 'arm-none-eabi-gcc was not found.'
}

$sourceFiles = Get-ChildItem -LiteralPath (Join-Path $repositoryRoot 'src') `
    -Recurse -Filter '*.c' | Select-Object -ExpandProperty FullName

$includeDirectories = @(
    'ra_cfg\fsp_cfg\bsp',
    'Debug',
    'ra_gen',
    'ra_cfg\fsp_cfg',
    'ra_cfg\aws',
    'src',
    'ra\fsp\inc',
    'ra\fsp\inc\api',
    'ra\fsp\inc\instances',
    'ra\fsp\src\rm_freertos_port',
    'ra\aws\FreeRTOS\FreeRTOS\Source\include',
    'ra\arm\CMSIS_6\CMSIS\Core\Include'
)

$compilerArguments = @(
    '-mthumb',
    '-mfloat-abi=hard',
    '-mcpu=cortex-m85+nopacbti',
    '-std=c99',
    '-fsyntax-only',
    '-Wall',
    '-Wextra',
    '-Werror',
    '-Wno-stringop-overflow',
    '-Wno-format-truncation',
    '-flax-vector-conversions',
    '--param=min-pagesize=0',
    '-D_RENESAS_RA_',
    '-D_RA_CORE=CPU0',
    '-D_RA_ORDINAL=1'
)

foreach ($includeDirectory in $includeDirectories)
{
    $compilerArguments += '-I' + (Join-Path $repositoryRoot $includeDirectory)
}

function New-CustomProfileDefines
{
    param(
        [uint32] $EscMode = 0,
        [uint32] $ControlMode = 0,
        [uint32] $PropLoadMode = 0,
        [uint32] $ImuDiagnosticMode = 0
    )

    $escEnabled = [uint32] ($EscMode -ne 0)
    $controlEnabled = [uint32] ($ControlMode -ne 0)
    $poweredControl = [uint32] ($ControlMode -eq 8)
    $propLoadEnabled = [uint32] ($PropLoadMode -ne 0)

    return @(
        '-DPROJECT_BUILD_PROFILE=2U',
        '-DPROJECT_DANGEROUS_BUILD_ACK=0x54455448UL',
        "-DESC_BENCH_MODE=${EscMode}U",
        "-DESC_BENCH_SAFETY_ACKNOWLEDGED=${escEnabled}U",
        "-DCONTROL_BENCH_MODE=${ControlMode}U",
        "-DCONTROL_BENCH_SAFETY_ACKNOWLEDGED=${controlEnabled}U",
        "-DCONTROL_BENCH_ESC_POWER_ACKNOWLEDGED=${poweredControl}U",
        "-DPROP_LOAD_TEST_MODE=${PropLoadMode}U",
        "-DPROP_LOAD_TEST_AIRFRAME_RESTRAINED_ACKNOWLEDGED=${propLoadEnabled}U",
        "-DPROP_LOAD_TEST_PROPELLERS_ACKNOWLEDGED=${propLoadEnabled}U",
        "-DPROP_LOAD_TEST_ESC_POWER_ACKNOWLEDGED=${propLoadEnabled}U",
        '-DTETHERED_FLIGHT_MODE=0U',
        '-DTETHERED_FLIGHT_RESTRAINT_ACKNOWLEDGED=0U',
        '-DTETHERED_FLIGHT_PROPELLERS_ACKNOWLEDGED=0U',
        '-DTETHERED_FLIGHT_AREA_AND_KILL_ACKNOWLEDGED=0U',
        '-DTETHERED_FLIGHT_ESC_POWER_ACKNOWLEDGED=0U',
        "-DIMU_DIAGNOSTIC_MODE=${ImuDiagnosticMode}U"
    )
}

$profiles = @(
    @{ Name = 'safe'; Defines = @() },
    @{
        Name = 'tethered_first_hop'
        Defines = @(
            '-DPROJECT_BUILD_PROFILE=1U',
            '-DPROJECT_DANGEROUS_BUILD_ACK=0x54455448UL'
        )
    },
    @{ Name = 'esc_calibration'; Defines = New-CustomProfileDefines -EscMode 1 },
    @{ Name = 'esc_sequence'; Defines = New-CustomProfileDefines -EscMode 2 },
    @{ Name = 'stick_mixer'; Defines = New-CustomProfileDefines -ControlMode 1 },
    @{ Name = 'imu_level'; Defines = New-CustomProfileDefines -ControlMode 2 },
    @{ Name = 'imu_rate'; Defines = New-CustomProfileDefines -ControlMode 3 },
    @{ Name = 'imu_cascade'; Defines = New-CustomProfileDefines -ControlMode 4 },
    @{ Name = 'rc_attitude'; Defines = New-CustomProfileDefines -ControlMode 5 },
    @{ Name = 'rc_yaw_rate'; Defines = New-CustomProfileDefines -ControlMode 6 },
    @{ Name = 'full_control'; Defines = New-CustomProfileDefines -ControlMode 7 },
    @{ Name = 'powered_control'; Defines = New-CustomProfileDefines -ControlMode 8 },
    @{ Name = 'shadow_control'; Defines = New-CustomProfileDefines -ControlMode 9 },
    @{ Name = 'pid_i_shadow'; Defines = New-CustomProfileDefines -ControlMode 10 },
    @{ Name = 'prop_load'; Defines = New-CustomProfileDefines -PropLoadMode 1 },
    @{ Name = 'imu_diagnostic'; Defines = New-CustomProfileDefines -ImuDiagnosticMode 1 }
)

foreach ($profile in $profiles)
{
    $profileArguments = $compilerArguments + $profile.Defines

    foreach ($sourceFile in $sourceFiles)
    {
        & $compiler @profileArguments $sourceFile

        if ($LASTEXITCODE -ne 0)
        {
            throw "Profile '$($profile.Name)' failed for '$sourceFile'."
        }
    }

    Write-Output "profile syntax passed: $($profile.Name)"
}
