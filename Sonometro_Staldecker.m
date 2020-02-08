function varargout = Sonometro_Staldecker(varargin)
% SONOMETRO_STALDECKER MATLAB code for Sonometro_Staldecker.fig
%      SONOMETRO_STALDECKER, by itself, creates a new SONOMETRO_STALDECKER or raises the existing
%      singleton*.
%
%      H = SONOMETRO_STALDECKER returns the handle to a new SONOMETRO_STALDECKER or the handle to
%      the existing singleton*.
%
%      SONOMETRO_STALDECKER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SONOMETRO_STALDECKER.M with the given input arguments.
%
%      SONOMETRO_STALDECKER('Property','Value',...) creates a new SONOMETRO_STALDECKER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Sonometro_Staldecker_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Sonometro_Staldecker_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Sonometro_Staldecker

% Last Modified by GUIDE v2.5 15-Nov-2017 20:47:35

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Sonometro_Staldecker_OpeningFcn, ...
                   'gui_OutputFcn',  @Sonometro_Staldecker_OutputFcn, ...
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


% --- Executes just before Sonometro_Staldecker is made visible.
function Sonometro_Staldecker_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Sonometro_Staldecker (see VARARGIN)

% Choose default command line output for Sonometro_Staldecker
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Sonometro_Staldecker wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Sonometro_Staldecker_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global dir_archivo_calibracion
dir_archivo_calibracion=[];
[nom_archivo,dir] = uigetfile('.wav');
dir_archivo_calibracion=strcat(dir,nom_archivo);
set (handles.path_calibracion,'String',dir_archivo_calibracion);


function path_calibracion_Callback(hObject, eventdata, handles)
% hObject    handle to path_calibracion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of path_calibracion as text
%        str2double(get(hObject,'String')) returns contents of path_calibracion as a double


% --- Executes during object creation, after setting all properties.
function path_calibracion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_calibracion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc;

direccion_archivo=get(handles.path_calibracion,'string')
[signal_calibracion,fs]= audioread(direccion_archivo);
%sound(signal_calibracion,fs)

global rms_calibracion
rms_calibracion= rms (signal_calibracion)

msgbox('Calibración exitosa');

function rms_Callback(hObject, eventdata, handles)
% hObject    handle to rms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of rms as text
%        str2double(get(hObject,'String')) returns contents of rms as a double


% --- Executes during object creation, after setting all properties.
function rms_CreateFcn(hObject, eventdata, handles)
% hObject    handle to rms (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global dir_archivo_signal
global y
global largo
global fs

[nom_archivo,dire] = uigetfile('.wav');

archivos = dir(dire);
largo=length(archivos)

%Se carga automáticamente la matriz de vectores de presion dentro de la
%carpeta seleccionada 

for u=1:largo-2

nombre=sprintf('muestra_%d.wav',u)
dir_archivo_signal=strcat(dire,nombre);
[y2,fs]=audioread(dir_archivo_signal);
y {u}=y2;
end

set (handles.path_signal,'String',dir_archivo_signal);
display(largo);

% Mensaje de aviso para el usuario

msgbox('Se analizarán todas las muestras de audio contenidas en la carpeta');

function path_signal_Callback(hObject, eventdata, handles)
% hObject    handle to path_signal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of path_signal as text
%        str2double(get(hObject,'String')) returns contents of path_signal as a double


% --- Executes during object creation, after setting all properties.
function path_signal_CreateFcn(hObject, eventdata, handles)
% hObject    handle to path_signal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

clc;
 
global rms_calibracion 
global dir_archivo_signal
global y
global largo
global fs
global leq_banda_oct
global leq_banda_tercio
global tercio
global octava
global leq_banda_oct_ponderado
global leq_banda_tercio_ponderado logger 
leq_banda_tercio=[];
leq_banda_oct=[];
signal=[];
signal_pascales=[];
logger=[];

tercio= get(handles.tercio,'Value');
octava= get(handles.octava,'Value');
ponderacion_a= get(handles.ponderacion_a,'Value');
ponderacion_c= get(handles.ponderacion_c,'Value');
ponderacion_z= get(handles.ponderacion_z,'Value');
slow= get(handles.slow,'Value')
fast= get(handles.fast,'Value')
impulse= get(handles.impulse,'Value')
fs_44100= get (handles. fs_44100, 'value');
fs_48000= get (handles. fs_48000, 'value');
tiempo= get(handles.tiempo,'value')

vec_pond_a_tercio=[-44.7 -39.4 -34.6 -30.2 -26.2 -22.5 -19.1 -16.1 -13.4 -10.9 -8.6 -6.6 -4.8 -3.2 -1.9 -0.8 0 0.6 1 1.2 1.3 1.2 1 0.5 -0.1 -1.1 -2.5 -4.3 -6.6 -9.3];
vec_pond_c_tercio=[-4.4 -3 -2 -1.3 -0.8 -0.5 -0.3 -0.2 -0.1 0 0 0 0 0 0 0 0 0 -0.1 -0.2 -0.3 -0.5 -0.8 -1.3 -2 -3 -4.4 -6.2 -8.5 -11.2];
vec_pond_a_octava=[-39.4 -26.2 -16.1 -8.6	-3.2 0 1.2	1 -1.1 -6.6];
vec_pond_c_octava=[-3 -0.8 -0.2 0 0 0 -0.2	-0.8 -3	-8.5];

% Asignación de cantidad de muestras por ventana de integración de acuerdo
% a la opción seleccionada 

if fs_44100==1
frecuencia_sampleo=44100;
muestras_slow=44100;
muestras_fast=5954;
muestras_impulse=1544;
else
    
frecuencia_sampleo=48000;
muestras_slow=48000;
muestras_fast=6480;
muestras_impulse=1680;
end

%Diseño de los filtros de tercio de octava bajo norma ANSI

   d = fdesign.octave(3,'Class 0', 'N,F0', 12, 1000, 44100);   
   F0=validfrequencies(d);      

for f=1:length(F0)      
    
    d.F0 = F0(f);
    F_ter(f) = design (d, 'butter'); 
end


%Análisis automático de todas las señales de presión medidas

for n=1:largo-2

% Se recorta la señal para valores menores al 1% del máximo registrado

umbral= max (y{n})/100;
signal = y{n}(find(y{n}>umbral,1,'first'):find(y{n}>umbral,1,'last'));
signal = signal';

%Se grafica la señal a analizar

plot(handles.grafico_signal,signal)
grid on
xlabel('Tiempo (seg)')
ylabel('Amplitud (dB)')

switch tiempo
    
    case 1
        tiempo_medicion=5;
        muestras_tiempo= fs*tiempo_medicion;
        if muestras_tiempo>length(signal)
            msgbox('Ingrese una muestra de audio de mayor duración o reduzca el tiempo de análisis','error')
            return
        else
        signal=signal(1:muestras_tiempo);
        end
        
    case 2
        tiempo_medicion=10;
        muestras_tiempo= fs*tiempo_medicion;
        if muestras_tiempo>length(signal)
            msgbox('Ingrese una muestra de audio de mayor duración o reduzca el tiempo de análisis','error')
            return
        else
        signal=signal(1:muestras_tiempo);
        end
        
    case 3
        tiempo_medicion=30;
        muestras_tiempo= fs*tiempo_medicion;
        if muestras_tiempo>length(signal)
            msgbox('Ingrese una muestra de audio de mayor duración o reduzca el tiempo de análisis','error')
            return
        else
        signal=signal(1:muestras_tiempo);
        end
    case 4
        tiempo_medicion=60;
        muestras_tiempo= fs*tiempo_medicion;
        if muestras_tiempo>length(signal)
            msgbox('Ingrese una muestra de audio de mayor duración o reduzca el tiempo de análisis','error')
            return
        else
        signal=signal(1:muestras_tiempo);
        end
    case 5
        tiempo_medicion=120;
        muestras_tiempo= fs*tiempo_medicion;
        if muestras_tiempo>length(signal)
            msgbox('Ingrese una muestra de audio de mayor duración o reduzca el tiempo de análisis','error')
            return
        else
        signal=signal(1:muestras_tiempo);
        end
        case 6
           signal=signal;            
        
end

%Conversión de valores de tensión a Pascales

signal_pascales= rdivide(signal, rms_calibracion);
   
if tercio==1
    
   display('tercio')   
     
   for i=1:length(F0)    
       
    sumatoria_banda=0;
    signal_filtrada_ter {i} = filter(F_ter(i),signal_pascales);
    a=1;
    if slow==1
        
        cant_ventanas= floor(length(signal)/muestras_slow);
        
        for j=1:cant_ventanas
            
            presion=0;
            presion_ventana=0;
            spl_ventana=0;            
            
           
            for k=a:(j*muestras_slow)
                
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+((signal_filtrada_ter {i}(k))^2);
                
                
            end
            presion_ventana=sqrt(presion/muestras_slow);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            logger(i,j)=spl_ventana;
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            a=(j*muestras_slow)+1;
            
        end        
            
        
    elseif fast ==1
        
        cant_ventanas= floor(length(signal)/muestras_fast);
        
        for j=1:cant_ventanas
           
            presion=0;
            presion_ventana=0;
            spl_ventana=0;
            spl_banda=0;
            
           
            for k=a:(j*muestras_fast)
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+(signal_filtrada_ter {i}(k))^2;
                              
            end
            presion_ventana=sqrt(presion/muestras_fast);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            logger(i,j)=spl_ventana;
            a=(j*muestras_fast)+1;
            
        end       
        
       
    else
        
        cant_ventanas= floor(length(signal)/muestras_impulse);
        
        for j=1:cant_ventanas
           
            presion=0;
            presion_ventana=0;
            spl_ventana=0;
            spl_banda=0;
            
           
            for k=a:(j*muestras_impulse)
                
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+(signal_filtrada_ter {i}(k))^2;
                                
            end
            presion_ventana=sqrt(presion/muestras_impulse);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            logger(i,j)=spl_ventana;
            a=(j*muestras_impulse)+1;
            
        end        
              
    end
    
    leq_banda_tercio{n}(i)= 10*log10(sumatoria_banda/cant_ventanas);
    
   end
   
   if ponderacion_a==1
    
    leq_banda_tercio_ponderado{n}= leq_banda_tercio{n}+vec_pond_a_tercio;
    
    elseif ponderacion_c==1
    
    leq_banda_tercio_ponderado{n}= leq_banda_tercio{n}+vec_pond_c_tercio;
    
    else
    leq_banda_tercio_ponderado{n}= leq_banda_tercio{n};
    
   end
   
     for g=1:length(leq_banda_tercio_ponderado{n})
       if leq_banda_tercio_ponderado{n}(g)<0      
       leq_banda_tercio_ponderado{n}(g)=0;
       end  
       
     end
   
   bar (handles.grafico,leq_banda_tercio_ponderado{n})
   grid on
   xlabel(handles.grafico,'Frecuencia (Hz)')
   ylabel(handles.grafico,'SPL(dB)')
   xlim([0 31])
   xticks([2 5 8 11 14 17 20 23 26 29])    
   set(handles.grafico,'XTickLabels',{'31.5' '63' '125' '250' '500' '1K' '2K' '4K' '8K' '16K'});
         
   
else
    
    display ('octava');      
    
    d = fdesign.octave(1,'Class 0', 'N,F0', 12, 1000, 44000);     
    F0=validfrequencies(d);
   
    for i=1:length(F0)   
    sumatoria_banda=0;    
    d.F0 = F0(i);
    F_oct(i) = design (d, 'butter'); 
    signal_filtrada_oct {i} = filter(F_oct(i),signal_pascales);   
    a=1;
    
    if slow==1
        
        cant_ventanas= floor(length(signal)/muestras_slow)
        
        for j=1:cant_ventanas
            
            presion=0;
            presion_ventana=0;
            spl_ventana=0;
            spl_banda=0;
            
           
            for k=a:(j*muestras_slow)
                
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+((signal_filtrada_oct {i}(k))^2);
                
                
            end
            presion_ventana=sqrt(presion/muestras_slow);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            logger(i,j)=spl_ventana;
            a=(j*muestras_slow)+1
            
        end        
            
        
    elseif fast ==1
        
        cant_ventanas= floor(length(signal)/muestras_fast);
        
        for j=1:cant_ventanas
           
            presion=0;
            presion_ventana=0;
            spl_ventana=0;
            spl_banda=0;
            
           
            for k=a:(j*muestras_fast)
                
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+(signal_filtrada_oct {i}(k))^2;
                              
            end
            presion_ventana=sqrt(presion/muestras_fast);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            logger(i,j)=spl_ventana;
            a=(j*muestras_fast)+1;
            
        end       
        
       
    else
        
        cant_ventanas= floor(length(signal)/muestras_impulse);
        
        for j=1:cant_ventanas
           
            presion=0;
            presion_ventana=0;
            spl_ventana=0;
            spl_banda=0;
            
           
            for k=a:(j*muestras_impulse)
                
                %se suman las presiones al cuadrado para el cálculo de la
                %presión eficaz de cada ventana               
                presion=presion+(signal_filtrada_oct {i}(k))^2;
                                
            end
            presion_ventana=sqrt(presion/muestras_impulse);
            spl_ventana=20*log10(presion_ventana/0.00002);            
            sumatoria_banda=sumatoria_banda+10^((spl_ventana)/10);
            logger(i,j)=spl_ventana;
            a=(j*muestras_impulse)+1;
            
        end        
              
    end
    
    leq_banda_oct{n}(i)= 10*log10(sumatoria_banda/cant_ventanas);
    
   end
   
   if ponderacion_a==1
    
    leq_banda_oct_ponderado{n}= leq_banda_oct{n}+vec_pond_a_octava;
    
    elseif ponderacion_c==1
    
    leq_banda_oct_ponderado{n}= leq_banda_oct{n}+vec_pond_c_octava;
    
    else
    leq_banda_oct_ponderado{n}= leq_banda_oct{n};
    
   end
   
   for g=1:length(leq_banda_oct_ponderado{n})
       if leq_banda_oct_ponderado{n}(g)<0      
       leq_banda_oct_ponderado{n}(g)=0;
       end  
       
   end
   
   
   bar (handles.grafico,leq_banda_oct_ponderado{n})
   grid on
   xlabel(handles.grafico,'Frecuencia (Hz)')
   ylabel(handles.grafico,'SPL(dB)')
   xticks([1 2 3 4 5 6 7 8 9 10])    
   set(handles.grafico,'XTickLabels',{'31.5' '63' '125' '250' '500' '1K' '2K' '4K' '8K' '16K'});    
   
   
end
 

end


%display (leq_banda_oct);

function Presion_Callback(hObject, eventdata, handles)
% hObject    handle to Presion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Presion as text
%        str2double(get(hObject,'String')) returns contents of Presion as a double


% --- Executes during object creation, after setting all properties.
function Presion_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Presion (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function spl_Callback(hObject, eventdata, handles)
% hObject    handle to spl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of spl as text
%        str2double(get(hObject,'String')) returns contents of spl as a double


% --- Executes during object creation, after setting all properties.
function spl_CreateFcn(hObject, eventdata, handles)
% hObject    handle to spl (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in tiempo.
function tiempo_Callback(hObject, eventdata, handles)
% hObject    handle to tiempo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns tiempo contents as cell array
%        contents{get(hObject,'Value')} returns selected item from tiempo


% --- Executes during object creation, after setting all properties.
function tiempo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tiempo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global leq_banda_oct
global leq_banda_tercio
global largo 
global tercio
global octava 
global leq_banda_tercio_ponderado
global leq_banda_oct_ponderado dir_archivo vec_logger largo_logx logger

[nombre_planilla,dire_planilla] = uigetfile('.xls');
dir_archivo=strcat(dire_planilla,nombre_planilla);

if tercio==1
    
    for m=1:largo-2
    
    celdas_1=sprintf('B%d:',m+1);
    celdas_2=sprintf('AE%d',m+1);
    celdas=strcat(celdas_1,celdas_2);
    xlswrite(dir_archivo,leq_banda_tercio_ponderado{m},'Tercio',celdas);
    end
else 
    for m=1:largo-2
      
    celdas_1=sprintf('B%d:',m+1);
    celdas_2=sprintf('K%d',m+1);
    celdas=strcat(celdas_1,celdas_2);
    xlswrite(dir_archivo,leq_banda_oct_ponderado{m},'Octava',celdas);
    end
end
logger_trpsd=logger';
vec_logger_trpsd=vec_logger';
xlswrite(dir_archivo,vec_logger_trpsd,'soft','AF3');
xlswrite(dir_archivo,logger_trpsd,'soft','B3');

% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


global logger vec_logger largo_logx

vec_logger=[];
largo_logx=length(logger(1,:))
largo_logy=length(logger(:,1))
for w=1:largo_logx
    
    suma_log=0

    for z=1:largo_logy    
    algo=10.^(logger(z,w)/10);    
    suma_log=suma_log+algo;
    end
    spl_log=10*log10(suma_log);
    vec_logger(w)=spl_log;
end
    
figure('name','Logger')
stem(vec_logger)

    
