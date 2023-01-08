#!/usr/bin/env pwsh

#Requires -PSEdition Core
#Requires -Version 7.2

param (
	[Parameter(Mandatory = $true, HelpMessage = "Toolchain to use")]
	[ValidateSet("gcc", "llvm")]
	[string]
	$Toolchain,

	[Parameter(Mandatory = $true, HelpMessage = "Major version of the toolchain")]
	[ValidatePattern("^\d+$")]
	[string]
	$Version
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

function Set-DefaultCommand($Name, $Path) {
	Write-Output "Displaying alternatives to '$Name' pre install..."

	update-alternatives --display $Name

	Write-Output "Installing alternatives to '$Name'..."

	update-alternatives --install /usr/bin/$Name $Name $Path 100
	update-alternatives --set $Name $Path

	Write-Output "Displaying alternatives to '$Name' post install..."

	update-alternatives --display $Name
}

Write-Output "Adding the ubuntu-toolchain-r repository..."

add-apt-repository --yes --update ppa:ubuntu-toolchain-r/ppa

if ($Toolchain -eq "gcc") {
	Write-Output "Installing g++-$Version-multilib..."

	apt install --quiet --yes g++-$Version-multilib

	Write-Output "Setting GCC $Version as the default..."

	Set-DefaultCommand -Name gcc -Path /usr/bin/gcc-$Version
	Set-DefaultCommand -Name g++ -Path /usr/bin/g++-$Version
}
elseif ($Toolchain -eq "llvm") {
	Write-Output "Installing g++-multilib..."

	apt install --quiet --yes g++-multilib

	Write-Output "Setting LLVM $Version as the default..."

	Set-DefaultCommand -Name clang -Path /usr/bin/clang-$Version
	Set-DefaultCommand -Name clang++ -Path /usr/bin/clang++-$Version
	Set-DefaultCommand -Name clang-format -Path /usr/bin/clang-format-$Version
	Set-DefaultCommand -Name clang-tidy -Path /usr/bin/clang-tidy-$Version
}
