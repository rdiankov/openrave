#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2011 Rosen Diankov (rosen.diankov@gmail.com)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
__author__ = 'Rosen Diankov'
__copyright__ = 'Copyright (C) 2009-2010 Rosen Diankov (rosen.diankov@gmail.com)'
__license__ = 'Apache License, Version 2.0'

from optparse import OptionParser
import os, sys, re, shutil, urllib
from types import ModuleType

EnvVarUpdate = """
/**
 *  EnvVarUpdate.nsh
 *    : Environmental Variables: append, prepend, and remove entries
 *
 *     WARNING: If you use StrFunc.nsh header then include it before this file
 *              with all required definitions. This is to avoid conflicts
 *
 *  Usage:
 *    ${EnvVarUpdate} "ResultVar" "EnvVarName" "Action" "RegLoc" "PathString"
 *
 *  Credits:
 *  Version 1.0 
 *  * Cal Turney (turnec2)
 *  * Amir Szekely (KiCHiK) and e-circ for developing the forerunners of this
 *    function: AddToPath, un.RemoveFromPath, AddToEnvVar, un.RemoveFromEnvVar,
 *    WriteEnvStr, and un.DeleteEnvStr
 *  * Diego Pedroso (deguix) for StrTok
 *  * Kevin English (kenglish_hi) for StrContains
 *  * Hendri Adriaens (Smile2Me), Diego Pedroso (deguix), and Dan Fuhry  
 *    (dandaman32) for StrReplace
 *
 *  Version 1.1 (compatibility with StrFunc.nsh)
 *  * techtonik
 *
 *  http://nsis.sourceforge.net/Environmental_Variables:_append%%2C_prepend%%2C_and_remove_entries
 *
 */


!ifndef ENVVARUPDATE_FUNCTION
!define ENVVARUPDATE_FUNCTION
!verbose push
!verbose 3
!include "LogicLib.nsh"
!include "WinMessages.NSH"
!include "StrFunc.nsh"

; ---- Fix for conflict if StrFunc.nsh is already includes in main file -----------------------
!macro _IncludeStrFunction StrFuncName
  !ifndef ${StrFuncName}_INCLUDED
    ${${StrFuncName}}
  !endif
  !ifndef Un${StrFuncName}_INCLUDED
    ${Un${StrFuncName}}
  !endif
  !define un.${StrFuncName} "${Un${StrFuncName}}"
!macroend

!insertmacro _IncludeStrFunction StrTok
!insertmacro _IncludeStrFunction StrStr
!insertmacro _IncludeStrFunction StrRep

; ---------------------------------- Macro Definitions ----------------------------------------
!macro _EnvVarUpdateConstructor ResultVar EnvVarName Action Regloc PathString
  Push "${EnvVarName}"
  Push "${Action}"
  Push "${RegLoc}"
  Push "${PathString}"
    Call EnvVarUpdate
  Pop "${ResultVar}"
!macroend
!define EnvVarUpdate '!insertmacro "_EnvVarUpdateConstructor"'
 
!macro _unEnvVarUpdateConstructor ResultVar EnvVarName Action Regloc PathString
  Push "${EnvVarName}"
  Push "${Action}"
  Push "${RegLoc}"
  Push "${PathString}"
    Call un.EnvVarUpdate
  Pop "${ResultVar}"
!macroend
!define un.EnvVarUpdate '!insertmacro "_unEnvVarUpdateConstructor"'
; ---------------------------------- Macro Definitions end-------------------------------------
 
;----------------------------------- EnvVarUpdate start----------------------------------------
!define hklm_all_users     'HKLM "SYSTEM\\CurrentControlSet\\Control\\Session Manager\\Environment"'
!define hkcu_current_user  'HKCU "Environment"'
 
!macro EnvVarUpdate UN
 
Function ${UN}EnvVarUpdate
 
  Push $0
  Exch 4
  Exch $1
  Exch 3
  Exch $2
  Exch 2
  Exch $3
  Exch
  Exch $4
  Push $5
  Push $6
  Push $7
  Push $8
  Push $9
  Push $R0
 
  /* After this point:
  -------------------------
     $0 = ResultVar     (returned)
     $1 = EnvVarName    (input)
     $2 = Action        (input)
     $3 = RegLoc        (input)
     $4 = PathString    (input)
     $5 = Orig EnvVar   (read from registry)
     $6 = Len of $0     (temp)
     $7 = tempstr1      (temp)
     $8 = Entry counter (temp)
     $9 = tempstr2      (temp)
     $R0 = tempChar     (temp)  */
 
  ; Step 1:  Read contents of EnvVarName from RegLoc
  ;
  ; Check for empty EnvVarName
  ${If} $1 == ""
    SetErrors
    DetailPrint "ERROR: EnvVarName is blank"
    Goto EnvVarUpdate_Restore_Vars
  ${EndIf}
 
  ; Check for valid Action
  ${If}    $2 != "A"
  ${AndIf} $2 != "P"
  ${AndIf} $2 != "R"
    SetErrors
    DetailPrint "ERROR: Invalid Action - must be A, P, or R"
    Goto EnvVarUpdate_Restore_Vars
  ${EndIf}
 
  ${If} $3 == HKLM
    ReadRegStr $5 ${hklm_all_users} $1     ; Get EnvVarName from all users into $5
  ${ElseIf} $3 == HKCU
    ReadRegStr $5 ${hkcu_current_user} $1  ; Read EnvVarName from current user into $5
  ${Else}
    SetErrors
    DetailPrint 'ERROR: Action is [$3] but must be "HKLM" or HKCU"'
    Goto EnvVarUpdate_Restore_Vars
  ${EndIf}
 
  ; Check for empty PathString
  ${If} $4 == ""
    SetErrors
    DetailPrint "ERROR: PathString is blank"
    Goto EnvVarUpdate_Restore_Vars
  ${EndIf}
 
  ; Make sure we've got some work to do
  ${If} $5 == ""
  ${AndIf} $2 == "R"
    SetErrors
    DetailPrint "$1 is empty - Nothing to remove"
    Goto EnvVarUpdate_Restore_Vars
  ${EndIf}
 
  ; Step 2: Scrub EnvVar
  ;
  StrCpy $0 $5                             ; Copy the contents to $0
  ; Remove spaces around semicolons (NOTE: spaces before the 1st entry or
  ; after the last one are not removed here but instead in Step 3)
  ${If} $0 != ""                           ; If EnvVar is not empty ...
    ${Do}
      ${${UN}StrStr} $7 $0 " ;"
      ${If} $7 == ""
        ${ExitDo}
      ${EndIf}
      ${${UN}StrRep} $0  $0 " ;" ";"         ; Remove '<space>;'
    ${Loop}
    ${Do}
      ${${UN}StrStr} $7 $0 "; "
      ${If} $7 == ""
        ${ExitDo}
      ${EndIf}
      ${${UN}StrRep} $0  $0 "; " ";"         ; Remove ';<space>'
    ${Loop}
    ${Do}
      ${${UN}StrStr} $7 $0 ";;" 
      ${If} $7 == ""
        ${ExitDo}
      ${EndIf}
      ${${UN}StrRep} $0  $0 ";;" ";"
    ${Loop}
 
    ; Remove a leading or trailing semicolon from EnvVar
    StrCpy  $7  $0 1 0
    ${If} $7 == ";"
      StrCpy $0  $0 "" 1                   ; Change ';<EnvVar>' to '<EnvVar>'
    ${EndIf}
    StrLen $6 $0
    IntOp $6 $6 - 1
    StrCpy $7  $0 1 $6
    ${If} $7 == ";"
     StrCpy $0  $0 $6                      ; Change ';<EnvVar>' to '<EnvVar>'
    ${EndIf}
    ; DetailPrint "Scrubbed $1: [$0]"      ; Uncomment to debug
  ${EndIf}
 
  /* Step 3. Remove all instances of the target path/string (even if "A" or "P")
     $6 = bool flag (1 = found and removed PathString)
     $7 = a string (e.g. path) delimited by semicolon(s)
     $8 = entry counter starting at 0
     $9 = copy of $0
     $R0 = tempChar      */
 
  ${If} $5 != ""                           ; If EnvVar is not empty ...
    StrCpy $9 $0
    StrCpy $0 ""
    StrCpy $8 0
    StrCpy $6 0
 
    ${Do}
      ${${UN}StrTok} $7 $9 ";" $8 "0"      ; $7 = next entry, $8 = entry counter
 
      ${If} $7 == ""                       ; If we've run out of entries,
        ${ExitDo}                          ;    were done
      ${EndIf}                             ;
 
      ; Remove leading and trailing spaces from this entry (critical step for Action=Remove)
      ${Do}
        StrCpy $R0  $7 1
        ${If} $R0 != " "
          ${ExitDo}
        ${EndIf}
        StrCpy $7   $7 "" 1                ;  Remove leading space
      ${Loop}
      ${Do}
        StrCpy $R0  $7 1 -1
        ${If} $R0 != " "
          ${ExitDo}
        ${EndIf}
        StrCpy $7   $7 -1                  ;  Remove trailing space
      ${Loop}
      ${If} $7 == $4                       ; If string matches, remove it by not appending it
        StrCpy $6 1                        ; Set 'found' flag
      ${ElseIf} $7 != $4                   ; If string does NOT match
      ${AndIf}  $0 == ""                   ;    and the 1st string being added to $0,
        StrCpy $0 $7                       ;    copy it to $0 without a prepended semicolon
      ${ElseIf} $7 != $4                   ; If string does NOT match
      ${AndIf}  $0 != ""                   ;    and this is NOT the 1st string to be added to $0,
        StrCpy $0 $0;$7                    ;    append path to $0 with a prepended semicolon
      ${EndIf}                             ;
 
      IntOp $8 $8 + 1                      ; Bump counter
    ${Loop}                                ; Check for duplicates until we run out of paths
  ${EndIf}
 
  ; Step 4:  Perform the requested Action
  ;
  ${If} $2 != "R"                          ; If Append or Prepend
    ${If} $6 == 1                          ; And if we found the target
      DetailPrint "Target is already present in $1. It will be removed and"
    ${EndIf}
    ${If} $0 == ""                         ; If EnvVar is (now) empty
      StrCpy $0 $4                         ;   just copy PathString to EnvVar
      ${If} $6 == 0                        ; If found flag is either 0
      ${OrIf} $6 == ""                     ; or blank (if EnvVarName is empty)
        DetailPrint "$1 was empty and has been updated with the target"
      ${EndIf}
    ${ElseIf} $2 == "A"                    ;  If Append (and EnvVar is not empty),
      StrCpy $0 $0;$4                      ;     append PathString
      ${If} $6 == 1
        DetailPrint "appended to $1"
      ${Else}
        DetailPrint "Target was appended to $1"
      ${EndIf}
    ${Else}                                ;  If Prepend (and EnvVar is not empty),
      StrCpy $0 $4;$0                      ;     prepend PathString
      ${If} $6 == 1
        DetailPrint "prepended to $1"
      ${Else}
        DetailPrint "Target was prepended to $1"
      ${EndIf}
    ${EndIf}
  ${Else}                                  ; If Action = Remove
    ${If} $6 == 1                          ;   and we found the target
      DetailPrint "Target was found and removed from $1"
    ${Else}
      DetailPrint "Target was NOT found in $1 (nothing to remove)"
    ${EndIf}
    ${If} $0 == ""
      DetailPrint "$1 is now empty"
    ${EndIf}
  ${EndIf}
 
  ; Step 5:  Update the registry at RegLoc with the updated EnvVar and announce the change
  ;
  ClearErrors
  ${If} $3  == HKLM
    WriteRegExpandStr ${hklm_all_users} $1 $0     ; Write it in all users section
  ${ElseIf} $3 == HKCU
    WriteRegExpandStr ${hkcu_current_user} $1 $0  ; Write it to current user section
  ${EndIf}
 
  IfErrors 0 +4
    MessageBox MB_OK|MB_ICONEXCLAMATION "Could not write updated $1 to $3"
    DetailPrint "Could not write updated $1 to $3"
    Goto EnvVarUpdate_Restore_Vars
 
  ; "Export" our change
  SendMessage ${HWND_BROADCAST} ${WM_WININICHANGE} 0 "STR:Environment" /TIMEOUT=5000
 
  EnvVarUpdate_Restore_Vars:
  ;
  ; Restore the user's variables and return ResultVar
  Pop $R0
  Pop $9
  Pop $8
  Pop $7
  Pop $6
  Pop $5
  Pop $4
  Pop $3
  Pop $2
  Pop $1
  Push $0  ; Push my $0 (ResultVar)
  Exch
  Pop $0   ; Restore his $0
 
FunctionEnd
 
!macroend   ; EnvVarUpdate UN
!insertmacro EnvVarUpdate ""
!insertmacro EnvVarUpdate "un."
;----------------------------------- EnvVarUpdate end----------------------------------------
 
!verbose pop
!endif
"""

nsiscript = """
# name the installer
!include "MUI2.nsh"
!include "StrFunc.nsh"
!include "Library.nsh"
%(EnvVarUpdate)s

Name "OpenRAVE %(openrave_version)s"
Caption "Open Robotics Automation Virtual Environment %(openrave_version)s for %(vsversion)s"
outFile "%(output_name)s.exe"

SetDateSave on
SetDatablockOptimize on
CRCCheck on
SilentInstall normal
BGGradient ecf8fa ffffff 112255
#InstallColors FF8080 000030
XPStyle on
SetCompress auto
#SetCompressor lzma
InstallDir "$PROGRAMFILES\\OpenRAVE-%(openrave_version)s"
AutoCloseWindow true          
SetOverwrite on

InstallDirRegKey HKLM "Software\\OpenRAVE" "InstallRoot"

RequestExecutionLevel admin

#LicenseData "%(license)s"
#Page license
#Page components
#Page directory
#Page instfiles
#UninstPage uninstConfirm
#UninstPage instfiles

!define MUI_ABORTWARNING
!insertmacro MUI_PAGE_LICENSE "%(license)s"
!insertmacro MUI_PAGE_COMPONENTS
!insertmacro MUI_PAGE_DIRECTORY
!insertmacro MUI_PAGE_INSTFILES
!insertmacro MUI_UNPAGE_CONFIRM
!insertmacro MUI_UNPAGE_INSTFILES
!insertmacro MUI_LANGUAGE "English"

Section  
  # check for boost installation
  ClearErrors
  ReadRegStr $0 HKLM "SOFTWARE\\boostpro.com\\%(boost_version)s" InstallRoot
  IfErrors 0 +11
  File "installers\\%(boost_installer)s"
  MessageBox MB_OK "Need to install boost %(boost_version)s"
  ExecWait '"$INSTDIR\\%(boost_installer)s"' $1
  Delete "$INSTDIR\\%(boost_installer)s"
  DetailPrint $1
  ClearErrors
  ReadRegStr $0 HKLM "SOFTWARE\\boostpro.com\\%(boost_version)s" InstallRoot
  IfErrors 0 +2
  MessageBox MB_OK "Failed to find boost %(boost_version)s"
  Abort "Cannot install"
    
SectionEnd

SectionGroup /e "PythonBindings" secpython
Section
  # check for boost installation
  ClearErrors
  ReadRegStr $0 HKLM "SOFTWARE\\Python\\PythonCore\\%(python_version)s\\PythonPath" ""
  IfErrors 0 +3
    MessageBox MB_OK "Failed to find python %(python_version)s installation"
    Abort "Cannot install"
SectionEnd

Section "Append Python Path"
  ${EnvVarUpdate} $0 "PYTHONPATH"  "A" "HKLM" "$INSTDIR\\share\\openrave"  
SectionEnd
SectionGroupEnd

Section /o "Register Octave Bindings" secoctave
SectionEnd

# default section start; every NSIS script has at least one section.
Section
  SetOutPath $INSTDIR  
  WriteRegStr HKLM SOFTWARE\\OpenRAVE\\%(openrave_version)s "InstallRoot" "$INSTDIR"
  
  File /r /x *.dll %(installdir)s\\bin 
  File /r %(installdir)s\\include
  File /r %(installdir)s\\lib
  File /r /x *.pyd %(installdir)s\\share

%(install_dll)s

  FileOpen $0 $INSTDIR\\include\\rave\\config.h w
  ${StrRep} $2 "$INSTDIR" "\\" "\\\\"
  ${StrRep} $1 "%(openrave_config)s" "__INSTDIR__" $2
  FileWrite $0 $1
  FileClose $0
  
  DetailPrint "boost installation at: $0"
  WriteUninstaller $INSTDIR\\uninstall.exe
SectionEnd

# Optional section (can be disabled by the user)
Section "Start Menu Shortcuts" secstart
  CreateDirectory "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s"
  CreateDirectory "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s\\examples"
  #CreateDirectory "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s\\databases"
  CreateShortCut "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s\\openrave.lnk" "$INSTDIR\\bin\\openrave.exe" "" "$INSTDIR\\bin\\openrave.exe" 0
  CreateShortCut "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s\\openravepy ipython.lnk" "$INSTDIR\\bin\\openrave.py" "-i" "$INSTDIR\\bin\\openrave.py" 0
  CreateShortCut "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s\\Uninstall.lnk" "$INSTDIR\\uninstall.exe" "" "$INSTDIR\\uninstall.exe" 0
  %(openrave_shortcuts)s
SectionEnd

Section "Append Environment Path" secpath
  ${EnvVarUpdate} $0 "Path"  "A" "HKLM" "$INSTDIR\\bin"  
SectionEnd

#Section Descriptions

;Language strings
LangString desc_secpython ${LANG_ENGLISH} "Installs Python bindings."
LangString desc_secoctave ${LANG_ENGLISH} "Installs Octave bindings."
LangString desc_secstart ${LANG_ENGLISH} "Installs start menu icons."
LangString desc_secpath ${LANG_ENGLISH} "Sets the environment path so OpenRAVE DLLs can be found."

#Assign language strings to sections
!insertmacro MUI_FUNCTION_DESCRIPTION_BEGIN
!insertmacro MUI_DESCRIPTION_TEXT ${secpython} $(desc_secpython)
!insertmacro MUI_DESCRIPTION_TEXT ${secoctave} $(desc_secoctave)
!insertmacro MUI_DESCRIPTION_TEXT ${secstart} $(desc_secstart)
!insertmacro MUI_DESCRIPTION_TEXT ${secpath} $(desc_secpath)
!insertmacro MUI_FUNCTION_DESCRIPTION_END

# create a section to define what the uninstaller does.
# the section will always be named "Uninstall"
Section "Uninstall"
  DeleteRegKey HKLM SOFTWARE\\OpenRAVE\\%(openrave_version)s
  ${un.EnvVarUpdate} $0 "PYTHONPATH"  "R" "HKLM" "$INSTDIR\\share\\openrave"
  ${un.EnvVarUpdate} $0 "Path"  "R" "HKLM" "$INSTDIR\\bin"

  # have to store install dir since it gets wiped out somewhere
  StrCpy $1 "$INSTDIR"
  
  # Always delete uninstaller first?
  Delete "$INSTDIR\\uninstall.exe"

%(uninstall_dll)s
  
  RMDir /r "$SMPROGRAMS\\OpenRAVE-%(openrave_version)s"
  # have to set current path outside of installation dir
  SetOutPath "$1\\.."
  RMDir /r "$1\\bin"
  RMDir /r "$1\\include"
  RMDir /r "$1\\lib"
  RMDir /r "$1\\share"
  RMDir "$1"
SectionEnd
"""

if __name__ == "__main__":
    parser = OptionParser(description='Creates a NSI installer for windows')
    parser.add_option('--lang',action="store",type='string',dest='lang',default='en',
                      help='Language folder.')
    parser.add_option('--installdir',action="store",type='string',dest='installdir',default=None,
                      help='Directory of the cmake installation')
    (options,args) = parser.parse_args()

    sys.path.insert(0,os.path.join(options.installdir,'share','openrave'))
    os.environ['PATH'] = os.path.join(os.path.abspath(options.installdir),'bin')+';'+os.environ['PATH']
    openravepy = __import__('openravepy')
    
    args = dict()
    args['openrave_version'] = openravepy.__version__
    args['vsversion'] = os.path.split(options.installdir)[1]
    args['openrave_shortcuts'] = ''
    args['output_name'] = 'openrave-%(openrave_version)s-win32-%(vsversion)s-setup'%args
    args['installdir'] = os.path.abspath(options.installdir)
    args['install_dll'] = ''
    args['uninstall_dll'] = ''
    args['EnvVarUpdate'] = EnvVarUpdate
    args['license'] = os.path.join(options.installdir,'share','openrave','LICENSE.lgpl')
    # install the dlls (allows us to use them without modifying the path)
    for dllname in os.listdir(os.path.join(options.installdir,'bin')):
        if os.path.splitext(dllname)[1] == '.dll':
            args['install_dll'] += '!insertmacro InstallLib DLL NOTSHARED NOREBOOT_PROTECTED %s\\bin\\%s $INSTDIR\\bin\\%s $INSTDIR\n'%(args['installdir'],dllname,dllname)
            args['uninstall_dll'] += '!insertmacro UninstallLib DLL NOTSHARED NOREBOOT_PROTECTED $INSTDIR\\bin\\%s\n'%(dllname)
    # python dlls
    for dllname in os.listdir(os.path.join(options.installdir,'share','openrave','openravepy')):
        if os.path.splitext(dllname)[1] == '.pyd':
            args['install_dll'] += '!insertmacro InstallLib DLL NOTSHARED NOREBOOT_PROTECTED %s\\share\\openrave\\openravepy\\%s $INSTDIR\\share\\openrave\\openravepy\\%s $INSTDIR\n'%(args['installdir'],dllname,dllname)
            args['uninstall_dll'] += '!insertmacro UninstallLib DLL NOTSHARED NOREBOOT_PROTECTED $INSTDIR\\share\\openrave\\openravepy\\%s\n'%(dllname)
    # add the runable examples 
    for name in dir(openravepy.examples):
        if not name.startswith('__'):
            try:
                m=__import__('openravepy.examples.'+name)
                if type(m) is ModuleType:
                    path = '$INSTDIR\\share\\openrave\\openravepy\\examples\\%s.py'%name
                    args['openrave_shortcuts'] += 'CreateShortCut "$SMPROGRAMS\\OpenRAVE-%s\\examples\\%s.lnk" "%s" "" "%s" 0\n'%(openravepy.__version__,name,path,path)
                    args['openrave_shortcuts'] += 'CreateShortCut "$SMPROGRAMS\\OpenRAVE-%s\\examples\\%s Documentation.lnk" "http://openrave.programmingvision.com/en/main/openravepy/examples.%s.html" "" "C:\WINDOWS\system32\shell32.dll" 979\n'%(openravepy.__version__,name,name)
            except ImportError:
                pass
    # not sure how useful this would be, perhaps a database generator GUI would help?
    for name in []:#dir(openravepy.databases):
        if not name.startswith('__'):
            try:
                m=__import__('openravepy.databases.'+name)
                if type(m) is ModuleType:
                    path = '$INSTDIR\\share\\openrave\\openravepy\\databases\\%s.py'%name
                    args['openrave_shortcuts'] += 'CreateShortCut "$SMPROGRAMS\\OpenRAVE-%s\\databases\\%s.lnk" "%s" "" "%s" 0\n'%(openravepy.__version__,name,path,path)
            except ImportError:
                pass

    # edit the config.h
    config = open(os.path.join(options.installdir,'include','rave','config.h'),'r').read()
    pattern=re.compile(args['installdir'].replace('\\','/'),re.IGNORECASE)
    args['openrave_config'] = pattern.sub('__INSTDIR__',config).replace('\n','$\\n').replace('"','$\\"').replace('\r','$\\r')
    open(os.path.join(options.installdir,'include','rave','config.h'),'w').write(config)
    
    # boost installation
    booststart = config.find('OPENRAVE_BOOST_VERSION "')+24
    boostend = config.find('"',booststart)
    args['boost_version'] = config[booststart:boostend]
    args['boost_installer'] = 'boost_%s_setup.exe'%args['boost_version'].replace('.','_')
    if not os.path.exists(os.path.join('installers',args['boost_installer'])):
        localfile, headers = urllib.urlretrieve('http://www.boostpro.com/download/'+args['boost_installer'])
        if headers['content-type'].find('application') < 0:
            raise ValueError('failed to find boost setup')
            
        try:
            os.mkdir('installers')
        except OSError:
            pass
            
        shutil.copyfile(localfile,os.path.join('installers',args['boost_installer']))
    
    # python installation
    args['python_version'] = '%s.%s'%(sys.version_info[0],sys.version_info[1])
    open(args['output_name']+'.nsi','w').write(nsiscript%args)
    os.system('"C:\\Program Files\\NSIS\\makensis.exe" %s.nsi'%args['output_name'])
  
def test():
    class empty: pass
    options = empty()
    options.installdir = 'C:\\jenkins\\workspace\\openrave_windows\\install\\vc10'
