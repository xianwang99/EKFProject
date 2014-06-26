%Copyright (c) 2014 Xian Wang.
%All rights reserved.

%Redistribution and use in source and binary forms are permitted
%provided that the above copyright notice and this paragraph are
%duplicated in all such forms and that any documentation,
%advertising materials, and other materials related to such
%distribution and use acknowledge that the software was developed
%by Xian Wang. The name of
%Xian Wang may not be used to endorse or promote products derived
%from this software without specific prior written permission.
%THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
%IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
%WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

function varargout = LocalizationGUI(varargin)
% LOCALIZATIONGUI MATLAB code for LocalizationGUI.fig
%      LOCALIZATIONGUI, by itself, creates a new LOCALIZATIONGUI or raises the existing
%      singleton*.
%
%      H = LOCALIZATIONGUI returns the handle to a new LOCALIZATIONGUI or the handle to
%      the existing singleton*.
%
%      LOCALIZATIONGUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LOCALIZATIONGUI.M with the given input arguments.
%
%      LOCALIZATIONGUI('Property','Value',...) creates a new LOCALIZATIONGUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before LocalizationGUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to LocalizationGUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help LocalizationGUI

% Last Modified by GUIDE v2.5 25-Jun-2014 15:26:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @LocalizationGUI_OpeningFcn, ...
                   'gui_OutputFcn',  @LocalizationGUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before LocalizationGUI is made visible.
function LocalizationGUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to LocalizationGUI (see VARARGIN)

clear Simulation;
clear FileManager;
clear handles.SIM;
clear handles.FM;

handles.SIM = Simulation();
handles.FM = FileManager();
% Choose default command line output for LocalizationGUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes LocalizationGUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = LocalizationGUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in StartEval.
function StartEval_Callback(hObject, eventdata, handles)
% hObject    handle to StartEval (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


command = get(handles.StartEval, 'String');
if(strcmpi(command, 'START simulation') == 1)
    
    if(handles.SIM.mSimulationFinished == 1)
        handles.FM.ReadDataFromFile(handles.SIM, handles.FM.mFilePath);
    end
    
    set(handles.StartEval, 'String', 'STOP Simulation');
    set(handles.OpenDataFileBtn, 'Enable', 'off');
    handles.SIM.SimulateRobot(handles);

else
    set(handles.StartEval, 'String', 'START Simulation');
    set(handles.OpenDataFileBtn, 'Enable', 'on');
    handles.SIM.StopSimulation();
end



% --- Executes on button press in OpenDataFileBtn.
function OpenDataFileBtn_Callback(hObject, eventdata, handles)
% hObject    handle to OpenDataFileBtn (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName,FilterIndex]  = uigetfile('.txt', 'Select Data File', 'Data');

if FileName == 0
    return
end

            
cla(handles.axes1);
cla(handles.axes2);
handles.FM.ReadDataFromFile(handles.SIM, strcat(PathName, FileName));
set(handles.StartEval, 'Visible', 'on');
