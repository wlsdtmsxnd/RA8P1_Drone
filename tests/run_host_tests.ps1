$ErrorActionPreference = 'Stop'

$testRoot = $PSScriptRoot
$repositoryRoot = (Resolve-Path -LiteralPath (Join-Path $testRoot '..')).Path
$buildRoot = Join-Path $testRoot 'build'
$controlTestExecutable = Join-Path $buildRoot 'control_math_tests.exe'
$upTofTestExecutable = Join-Path $buildRoot 'up_tof_protocol_tests.exe'

New-Item -ItemType Directory -Path $buildRoot -Force | Out-Null

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I (Join-Path $repositoryRoot 'src') `
    (Join-Path $testRoot 'test_control_math.c') `
    (Join-Path $repositoryRoot 'src\code\pid_controller.c') `
    (Join-Path $repositoryRoot 'src\code\quad_x_mixer.c') `
    -o $controlTestExecutable

if ($LASTEXITCODE -ne 0)
{
    throw 'Host test compilation failed.'
}

& $controlTestExecutable

if ($LASTEXITCODE -ne 0)
{
    throw 'Host control-math tests failed.'
}

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -I (Join-Path $repositoryRoot 'src') `
    (Join-Path $testRoot 'test_up_tof_protocol.c') `
    (Join-Path $repositoryRoot 'src\driver\up_tof_protocol.c') `
    -o $upTofTestExecutable

if ($LASTEXITCODE -ne 0)
{
    throw 'UPIX UP-T301 protocol test compilation failed.'
}

& $upTofTestExecutable

if ($LASTEXITCODE -ne 0)
{
    throw 'UPIX UP-T301 protocol tests failed.'
}
