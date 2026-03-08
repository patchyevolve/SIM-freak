; simPUS Installer Script for Inno Setup
; =============================================================================

#define MyAppName "simPUS"
#define MyAppVersion "1.0"
#define MyAppPublisher "simPUS Team"
#define MyAppExeName "simPUS.exe"
#define SourcePath "simPUS\x64\Release"

[Setup]
; NOTE: The value of AppId uniquely identifies this application. Do not use the same AppId value in installers for other applications.
; (To generate a new GUID, click Tools | Generate GUID inside the IDE.)
AppId={{C7A9E1B2-8D4F-4E2C-A1B2-C7A9E1B28D4F}
AppName={#MyAppName}
AppVersion={#MyAppVersion}
;AppVerName={#MyAppName} {#MyAppVersion}
AppPublisher={#MyAppPublisher}
DefaultDirName={autopf}\{#MyAppName}
DisableProgramGroupPage=yes
; Remove the following line to run in administrative install mode (install for all users.)
PrivilegesRequired=lowest
OutputBaseFilename=simPUS_Setup
Compression=lzma
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=6.1sp1
CloseApplications=yes
UninstallDisplayIcon={app}\{#MyAppExeName}

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked

[Files]
; Main Executable
Source: "{#SourcePath}\{#MyAppExeName}"; DestDir: "{app}"; Flags: ignoreversion

; SFML DLLs
Source: "{#SourcePath}\sfml-graphics-2.dll"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourcePath}\sfml-window-2.dll";   DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourcePath}\sfml-system-2.dll";   DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourcePath}\sfml-audio-2.dll";    DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourcePath}\sfml-network-2.dll";  DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourcePath}\openal32.dll";        DestDir: "{app}"; Flags: ignoreversion

; Shader Assets (Recursively copy the render folder)
Source: "{#SourcePath}\render\*"; DestDir: "{app}\render"; Flags: ignoreversion recursesubdirs createallsubdirs

; Optional: Sample Save File
Source: "simsus_test_save.json"; DestDir: "{app}"; Flags: ignoreversion

; Documentation (from project root)
Source: "README.md";           DestDir: "{app}"; Flags: ignoreversion
Source: "QUICKSTART.md";       DestDir: "{app}"; Flags: ignoreversion
Source: "ProjectHandbook.md";  DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{autoprograms}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"
Name: "{autodesktop}\{#MyAppName}";  Filename: "{app}\{#MyAppExeName}"; Tasks: desktopicon

[Run]
Filename: "{app}\{#MyAppExeName}"; Description: "{cm:LaunchProgram,{#StringChange(MyAppName, '&', '&&')}}"; Flags: nowait postinstall skipifsilent
