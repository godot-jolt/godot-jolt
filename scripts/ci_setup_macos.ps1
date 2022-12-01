#!/usr/bin/env pwsh

param (
	[Parameter(Mandatory = $true, HelpMessage = "Version of Xcode")]
	[ValidatePattern("^\d+\.\d+$")]
	[string]
	$Version
)

. $PSScriptRoot/_common.ps1

Write-Output "Setting Xcode $Version as the default..."

xcode-select --switch /Applications/Xcode_$Version.app
