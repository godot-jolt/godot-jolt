#!/usr/bin/env pwsh

param (
	[Parameter(Mandatory = $true, HelpMessage = "Path to directory with source files")]
	[ValidateNotNullOrEmpty()]
	[string]$SourcePath,

	[Parameter(Mandatory = $true, HelpMessage = "Path to directory with compile_commands.json")]
	[ValidateNotNullOrEmpty()]
	[string]$BuildPath,

	[Parameter(HelpMessage = "Apply fixes if applicable (warning: slow)")]
	[switch]$Fix = $false
)

. $PSScriptRoot/_common.ps1

$SourceFiles = Get-ChildItem -Recurse -Path $SourcePath -Include "*.cpp"

if ($Fix) {
	clang-tidy -p $BuildPath --quiet --fix-notes @SourceFiles
	exit $LASTEXITCODE
}

$Outputs = [Collections.Concurrent.ConcurrentBag[psobject]]::new()

$SourceFiles | ForEach-Object -Parallel {
	$BuildPath = $using:BuildPath
	$Outputs = $using:Outputs
	$Output = $null
	$($Output = clang-tidy -p $BuildPath --quiet $_ *>&1) || $Outputs.Add($Output)
} -ThrottleLimit ([Environment]::ProcessorCount)

$Outputs | Where-Object { $_ -ne $null } | ForEach-Object {
	Write-Output $_
	Write-Output ""
}

exit $Outputs.IsEmpty ? 0 : 1
