# name the installer
Name "OpenRAVE Installer"
Caption "This will install the Open Robotics Automation Virtual Environment (http://openrave.org)"
outFile "openrave-0.2.20-win32-vs2010-setup.exe"

SetDateSave on
SetDatablockOptimize on
CRCCheck on
SilentInstall normal
BGGradient 000000 800000 FFFFFF
InstallColors FF8080 000030
XPStyle on

InstallDir "$PROGRAMFILES\openrave-$OPENRAVE_VERSION"

; Registry key to check for directory (so if you install again, it will 
; overwrite the old one automatically)
InstallDirRegKey HKLM "Software\OpenRAVE" "Install_Dir"

#InstallDirRegKey HKLM "Software\NSISTest\BigNSISTest" "Install_Dir"

#LicenseText "A test text, make sure it's all there"
LicenseData "../LICENSE.lgpl"

RequestExecutionLevel user#admin

Page license
Page components
Page directory
Page instfiles

UninstPage uninstConfirm
UninstPage instfiles

# default section start; every NSIS script has at least one section.
Section "First Section"

  SetOutPath $INSTDIR

  ; Write the installation path into the registry
  WriteRegStr HKLM SOFTWARE\OpenRAVE "Install_Dir" "$INSTDIR"
  
  File ..\..\install\vc10

  WriteUninstaller $INSTDIR\uninstaller.exe

    
# default section end
SectionEnd

; Optional section (can be disabled by the user)
Section "Start Menu Shortcuts"
  CreateDirectory "$SMPROGRAMS\OpenRAVE"
  CreateShortCut "$SMPROGRAMS\OpenRAVE\Uninstall.lnk" "$INSTDIR\uninstall.exe" "" "$INSTDIR\uninstall.exe" 0
  #CreateShortCut "$SMPROGRAMS\Example2\Example2 (MakeNSISW).lnk" "$INSTDIR\example2.nsi" "" "$INSTDIR\example2.nsi" 0
SectionEnd


# create a section to define what the uninstaller does.
# the section will always be named "Uninstall"
Section "Uninstall"
  ; Remove registry keys
  DeleteRegKey HKLM SOFTWARE\OpenRAVE

  # Always delete uninstaller first
  Delete $INSTDIR\uninstall.exe

  ; Remove shortcuts, if any
  Delete "$SMPROGRAMS\OpenRAVE\*.*"

  ; Remove directories used
  RMDir "$SMPROGRAMS\OpenRAVE"
  RMDir "$INSTDIR"
SectionEnd
