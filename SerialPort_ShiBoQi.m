function varargout = SerialPort_ShiBoQi(varargin)
% SERIALPORT_SHIBOQI MATLAB code for SerialPort_ShiBoQi.fig
%      SERIALPORT_SHIBOQI, by itself, creates a new SERIALPORT_SHIBOQI or raises the existing
%      singleton*.
%
%      H = SERIALPORT_SHIBOQI returns the handle to a new SERIALPORT_SHIBOQI or the handle to
%      the existing singleton*.
%
%      SERIALPORT_SHIBOQI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SERIALPORT_SHIBOQI.M with the given input arguments.
%
%      SERIALPORT_SHIBOQI('Property','Value',...) creates a new SERIALPORT_SHIBOQI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SerialPort_ShiBoQi_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SerialPort_ShiBoQi_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SerialPort_ShiBoQi

% Last Modified by GUIDE v2.5 19-Jul-2019 21:34:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SerialPort_ShiBoQi_OpeningFcn, ...
                   'gui_OutputFcn',  @SerialPort_ShiBoQi_OutputFcn, ...
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


% --- Executes just before SerialPort_ShiBoQi is made visible.
function SerialPort_ShiBoQi_OpeningFcn(hObject, ~, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SerialPort_ShiBoQi (see VARARGIN)

% Choose default command line output for SerialPort_ShiBoQi
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SerialPort_ShiBoQi wait for user response (see UIRESUME)
% uiwait(handles.figure1);

%% 串口默认数据初始化
    set(handles.OpenSerialPort,'string','打开串口');
    set(handles.COM_Number,'enable','on','value',8);
    set(handles.BaudRate,'enable','on','value',10);
    set(handles.DataBit,'enable','on','value',4);
    set(handles.CheckType,'enable','on');
    set(handles.StopBit,'enable','on');
    set(handles.DataType,'enable','on');
        if isempty(instrfind) == 0
        fclose(instrfind);
        end
    
%% 默认窗口初始化，有些窗口需要初始化
                        set(handles.BTWS_Set,'visible','off');
                        set(handles.QBXF_Set,'visible','off');
                        set(handles.FIR_Set,'visible','off');
                        set(handles.EllipticFilter_Set,'visible','off');
                        set(handles.Kalman_Set,'visible','off');
                        set(handles.FilterStartButton,'visible','off');
                        set(handles.FilterSelect,'enable','on');
                        global CurrentFilterType
                        CurrentFilterType=handles.BTWS_Set;
 
%% 开启时钟，定时1秒,（关闭），少使用几个定时器，不然会有点卡
%     global  Time_Clock
%     Time_Clock=timer;
%     set(Time_Clock,'ExecutionMode','FixedRate');
%     set(Time_Clock,'Period',1);
%     set(Time_Clock,'TimerFcn',{@dispnow,handles});
%     start(Time_Clock);


 %% 使能傅里叶分析定时器，定时0.5秒，每0.5秒进行一次FFT。
    global  FFT_Time
    FFT_Time=timer;
    set(FFT_Time,'ExecutionMode','FixedRate');
    set(FFT_Time,'Period',1);
    set(FFT_Time,'TimerFcn',{@FFT,handles});
    start(FFT_Time);
    
%% 重要数据初始化
%% 串口接收数据的容器初始化,六个通道,每个通道存储接收到的数据
global channel
global DataBufferSize
DataBufferSize=20000;   % 数据缓存区大小，有关绘图快慢，FFT变换快慢
channel= struct('y',{zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize)} ...
                              ,'y_filter',{zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize)} ...
                              );
 


% --- Executes during object deletion, before destroying properties.
function figure1_DeleteFcn(~, ~, ~)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    if isempty(instrfind) == 0
    fclose(instrfind);
    end
%     global Time_Clock
%     stop(Time_Clock);
%     delete(Time_Clock);
    global FFT_Time
    stop(FFT_Time);
    delete(FFT_Time);
    
    
% --- Outputs from this function are returned to the command line.
function varargout = SerialPort_ShiBoQi_OutputFcn(~, ~, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


function  dispnow(~,~,handles)
set(handles.Time,'string',datestr(now));

%% ------定时器中断处理函数，用于FFT计算和显示频谱图
% 如果选择"通道一",则对通道一的原始数据计算频谱
% 如果选择"通道一滤波",则对通道一滤波后的数据计算频谱
function FFT(~,~,handles)
str=get(handles.filter_button,'string');
strtamp=0;
num=get(handles.FFTchannel,'value');
str_channelTemp=get(handles.FFTchannel,'string');
str_channel=char(str_channelTemp(num));
global channel
if strcmp(str,'STOP') == 1
   strtamp=get(handles.N,'string');
   N=str2num(strtamp);
   strtamp=get(handles.Fs,'string');
   Fs=str2num(strtamp);
   if strcmp(str_channel,'通道一') == 1
   data=fft(channel(1).y,N);
   else if strcmp(str_channel,'通道二') == 1
       else if strcmp(str_channel,'通道三') == 1
           else if strcmp(str_channel,'通道四') == 1
               else if strcmp(str_channel,'通道五') == 1
                   else if strcmp(str_channel,'通道一滤波') == 1
                              data=fft(channel(1).y_filter,N);
                       else if strcmp(str_channel,'通道二滤波') == 1
                           else if strcmp(str_channel,'通道三滤波') == 1
                               else if strcmp(str_channel,'通道四滤波') == 1
                                   else if strcmp(str_channel,'通道五滤波') == 1
                                       end
                                   end
                               end
                           end
                       end
                   end
               end
           end
       end
   end
   Pyy=abs(data)/(N/2);
   f=linspace(0,Fs/2,N/2);
   plot(handles.axes2,f,Pyy(1:(N/2)));
   xlabel(handles.axes2,'频率：f/Hz');
   ylabel(handles.axes2,'幅度');
   grid(handles.axes2);
   set(handles.axes2,'YAxisLocation','Right');
   xlim(handles.axes2,[0,Fs/4]);
end

%% COM口---中断回调函数
function COM_Number_Callback(~, ~, ~)
%% 数据采样率Fs---中断回调函数
function Fs_Callback(~, ~, ~)
%% FFT变换点数N---中断回调函数
function N_Callback(~, ~, ~)


% --- Executes during object creation, after setting all properties.
function COM_Number_CreateFcn(hObject, ~, ~)
% hObject    handle to COM_Number (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function BaudRate_CreateFcn(hObject, ~, ~)
% hObject    handle to BaudRate (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function DataBit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DataBit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function CheckType_CreateFcn(hObject, eventdata, ~)
% hObject    handle to CheckType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function StopBit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to StopBit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

%% 比较重要的一个中断回调函数,"打开串口"按钮中断回调
% --- Executes on button press in OpenSerialPort.
function OpenSerialPort_Callback(hObject, eventdata, handles)
% hObject    handle to OpenSerialPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global SerialPortObj
str=get(handles.OpenSerialPort,'string');
str_value='none';
num=0;
if str == '打开串口'
    %% 获取串口初始化必要信息---COM口
    COM=get(handles.COM_Number,'string');
    num=get(handles.COM_Number,'value');
    COM=char(COM(num));
    %% 获取串口初始化必要信息---波特率
    str_value=get(handles.BaudRate,'string');
    num=get(handles.BaudRate,'value');
    BaudRate=str2double(char(str_value(num)));
    %% 获取串口初始化必要信息---数据位
    str_value=get(handles.DataBit,'string');
    num=get(handles.DataBit,'value');
    DataBit=str2double(char(str_value(num)));
    %% 获取串口初始化必要信息---校验位
    CheckType=get(handles.CheckType,'string');
    num=get(handles.CheckType,'value');
    CheckType=char(CheckType(num));
    %% 获取串口初始化必要信息---停止位
    str_value=get(handles.StopBit,'string');
    num=get(handles.StopBit,'value');
    StopBit=str2double(char(str_value(num)));
    %% 获取串口初始化必要信息---传输数据类型
    str_value=get(handles.DataType,'string');
    num=get(handles.DataType,'value');
    global DataType 
    DataType=char(str_value(num));
    %% 读取DataNumber的初始化值，DataNumber决定多少个数据触发串口接收中断
    global DataNumber
    str_value=get(handles.DataNumberInit,'string');
    num=get(handles.DataNumberInit,'value');
    DataNumber=str2double(char(str_value(num)));
    %%  输入缓存区溢出计数初始化，及其膨胀。计算出接收多少个Byte触发串口接收中断
    global BytesAvailableFcnCount  %输入缓存区溢出计数初始化
    if (strcmp(DataType,'int8') || strcmp(DataType,'uint8')) ==1
        %如果数据是以uint8或者int8形式发送，不对DataNumber进行膨胀
        BytesAvailableFcnCount=DataNumber;
    else if (strcmp(DataType,'int16') || strcmp(DataType,'uint16')) ==1
            BytesAvailableFcnCount=DataNumber*2;
            %如果数据是以uint16或者int16形式发送，将DataNumber膨胀为原来两倍
        else if (strcmp(DataType,'int32') || strcmp(DataType,'uint32') || strcmp(DataType,'single')) ==1
                %如果数据是以uint32,single(即float)或者int32形式发送，将DataNumber膨胀为原来四倍
                BytesAvailableFcnCount=DataNumber*4;
            else if strcmp(DataType,'double')==1
                    %如果数据是以double形式发送，将DataNumber膨胀为原来八倍
                    BytesAvailableFcnCount=DataNumber*8;
                end
            end
        end
    end
     %%  串口初始化
    Axes1Handles=handles.axes1;
    FilterStartHandles=handles.FilterStartButton;
    Axes3Handles=handles.axes3;
    SerialPortObj = serial(COM ...
                             ,'BaudRate',BaudRate ...
                             ,'DataBits',DataBit ...
                             ,'StopBits',StopBit ...
                             ,'parity',CheckType ...
                             ,'InputBufferSize',4096 ...
                             ,'TimerPeriod',0.6 ...
                             ,'BytesAvailableFcnCount',BytesAvailableFcnCount ...
                             ,'BytesAvailableFcnMode','byte' ...
                             ,'BytesAvailableFcn', {@DataComingCallback,Axes1Handles,FilterStartHandles,Axes3Handles} ...
                             );
     fopen(SerialPortObj);
     %% 串口初始化成功后，进行窗口除能
    set(handles.OpenSerialPort,'string','关闭串口');
    set(handles.COM_Number,'enable','off');
    set(handles.BaudRate,'enable','off');
    set(handles.DataBit,'enable','off');
    set(handles.CheckType,'enable','off');
    set(handles.StopBit,'enable','off');
    set(handles.DataType,'enable','off');
    set(handles.DataNumberInit,'enable','off');
    reset(handles.axes1);
    cla(handles.axes1);
    cla(handles.axes2);
    cla(handles.axes3);
    global channel
    global DataBufferSize
    channel= struct('y',{zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize)} ...
                                  ,'y_filter',{zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize),zeros(1,DataBufferSize)} ...
                                  );
    global LineXdataCount  %line绘图使用的变量
    LineXdataCount=1;
else if str=='关闭串口'
        %% 串口关闭后窗口使能
            fclose(SerialPortObj);
            set(handles.OpenSerialPort,'string','打开串口');
            set(handles.COM_Number,'enable','on');
            set(handles.BaudRate,'enable','on');
            set(handles.DataBit,'enable','on');
            set(handles.CheckType,'enable','on');
            set(handles.StopBit,'enable','on');
            set(handles.DataType,'enable','on');
            set(handles.DataNumberInit,'enable','on');
    end
end

%% -----串口中断处理函数
function DataComingCallback(hObject, eventdata, Axes1Handles,FilterStartHandles,Axes3Handles)
global SerialPortObj
global channel
global DataBufferSize
global GlobalDigitalFilterb_z
global GlobalDigitalFiltera_z
global Dataout
global LineXdataCount
global DataType
global DataNumber
a_z=GlobalDigitalFiltera_z;
b_z=GlobalDigitalFilterb_z;
Array_Count=DataNumber;
LineXdataCount=LineXdataCount+Array_Count;
%% 缓存区更新以及axes1显示刷新
channel(1).y(1+Array_Count : DataBufferSize)=channel(1).y(1:(DataBufferSize-Array_Count));
channel(1).y(1:Array_Count)=fliplr(fread(SerialPortObj,Array_Count,DataType)');%接收到数据记得反向
y=channel(1).y(1:2048);
%% line 绘图,不知为何，
% 画完图之后很卡。所以不使用这种方式，
% 但是这种方式有优点，可以显示所有的数据
%{
m=LineXdataCount : -1 : (LineXdataCount-Array_Count-1);
num=length(m);
if LineXdataCount>2000
   xlim(Axes1Handles,[LineXdataCount-1500,LineXdataCount]);
end
if mod(num,2) == 0   %一次画偶数个点
    for k=0:1:(num/2)
        if k~=(num/2)
        line(Axes1Handles,'xdata',m(2*k+1 : 2*k+2) ...
                                         ,'ydata', y(2*k+1 : 2*k+2) ...
                                         ,'LineStyle','-');
        end
        if k~=(num/2) && k~=(num/2)-1
        line(Axes1Handles,'xdata',m(2*k+2 : 2*k+3) ...
                                         ,'ydata', y(2*k+2 : 2*k+3) ...
                                         ,'LineStyle','-');
        end
    end
else    %一次画奇数个点
    
end
set(Axes1Handles,'XDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
%}

%% plot绘图
plot(Axes1Handles,y,'-');
xlim(Axes1Handles,[1,1024]);


%% 不管哪一种绘图方式以下代码都要执行
grid(Axes1Handles);
%zoom(Axes1Handles,'vertical');
set(Axes1Handles,'YAxisLocation','Right');  

%% 串口数据滤波
str=get(FilterStartHandles,'string');
if strcmp(str,'滤波停止')==1
channel(1).y_filter=filter(b_z,a_z,channel(1).y);
y=channel(1).y_filter(1:DataBufferSize);
Dataout=[Dataout;channel(1).y_filter(DataBufferSize)];
plot(Axes3Handles,y,'-');
xlim(Axes3Handles,[1,1024]);
grid(Axes3Handles);
set(Axes3Handles,'YAxisLocation','Right');   
end




% --- Executes during object creation, after setting all properties.
function Fs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end




% --- Executes during object creation, after setting all properties.
function N_CreateFcn(hObject, eventdata, handles)
% hObject    handle to N (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function FilterSelect_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FilterSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes during object creation, after setting all properties.
function FFTchannel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FFTchannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over OpenSerialPort.
function OpenSerialPort_ButtonDownFcn(~, eventdata, handles)
% hObject    handle to OpenSerialPort (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Menu_Filter_Callback(hObject, eventdata, handles)
% hObject    handle to Menu_Filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



% --------------------------------------------------------------------
function Filter_Callback(hObject, eventdata, handles)
% hObject    handle to Filter (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
grid on
zoom on
pan on %拖拽曲线
datacursormode on % 数据光标模式
box off
set(gca,'YAxisLocation','Right');   
% Hint: place code in OpeningFcn to populate axes1


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
grid on
zoom on
pan on %拖拽曲线
datacursormode on % 数据光标模式
box off
set(gca,'YAxisLocation','Right');   
% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
grid on
zoom on
pan on %拖拽曲线
%datacursormode on % 数据光标模式
box off
set(gca,'YAxisLocation','Right');   
% Hint: place code in OpeningFcn to populate axes3


% --- Executes on mouse press over axes background.
function axes1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
grid on
zoom on
% pan on %拖拽曲线
% datacursormode on % 数据光标模式
box off


% --- Executes on button press in filter_button.
function filter_button_Callback(hObject, eventdata, handles)
% hObject    handle to filter_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%% 窗口使能与除能以及静态文本更新
str = get(handles.filter_button,'string');
if strcmp(str,'RUN') == 1
    set(handles.filter_button,'string','STOP');
    set(handles.N,'enable','off');
    set(handles.Fs,'enable','off');
else if strcmp(str,'STOP') == 1
    set(handles.filter_button,'string','RUN');
    set(handles.N,'enable','on');
    set(handles.Fs,'enable','on');
    end
end


% --- Executes on selection change in FilterSelect.
function FilterSelect_Callback(hObject, eventdata, handles)
% hObject    handle to FilterSelect (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns FilterSelect contents textbtwsas cell array
%        contents{get(hObject,'Value')} returns selected item from FilterSelect
%% 窗口使能与除能以及当前滤波器型号句柄参数更新，及静态文本更新
num=get(handles.FilterSelect,'value');
strtemp=get(handles.FilterSelect,'string');
str=char(strtemp(num));
global CurrentFilterType
if strcmp(str,'(空)')==0
    set(handles.FilterStartButton,'visible','on');
    set(handles.FilterStartButton,'string','滤波开始');
end
if strcmp(str,'巴特沃斯滤波器')==1
    CurrentFilterType=handles.BTWS_Set;
    set(handles.BTWS_Set,'visible','on');
    set(handles.QBXF_Set,'visible','off');
    set(handles.FIR_Set,'visible','off');
    set(handles.EllipticFilter_Set,'visible','off');
    set(handles.Kalman_Set,'visible','off');
else if strcmp(str,'切比雪夫滤波器')==1
            CurrentFilterType=handles.QBXF_Set;
            set(handles.BTWS_Set,'visible','off');
            set(handles.QBXF_Set,'visible','on');
            set(handles.FIR_Set,'visible','off');
            set(handles.EllipticFilter_Set,'visible','off');
            set(handles.Kalman_Set,'visible','off');
    else if strcmp(str,'FIR滤波器')==1
                CurrentFilterType=handles.FIR_Set;
                set(handles.BTWS_Set,'visible','off');
                set(handles.QBXF_Set,'visible','off');
                set(handles.FIR_Set,'visible','on');
                set(handles.EllipticFilter_Set,'visible','off');
                set(handles.Kalman_Set,'visible','off');  
        else if strcmp(str,'椭圆滤波器')==1
                    CurrentFilterType=handles.EllipticFilter_Set;
                    set(handles.BTWS_Set,'visible','off');
                    set(handles.QBXF_Set,'visible','off');
                    set(handles.FIR_Set,'visible','off');
                    set(handles.EllipticFilter_Set,'visible','on');
                    set(handles.Kalman_Set,'visible','off');
            else if strcmp(str,'卡尔曼滤波器')==1
                        CurrentFilterType=handles.Kalman_Set;
                        set(handles.BTWS_Set,'visible','off');
                        set(handles.QBXF_Set,'visible','off');
                        set(handles.FIR_Set,'visible','off');
                        set(handles.EllipticFilter_Set,'visible','off');
                        set(handles.Kalman_Set,'visible','on');
                else if strcmp(str,'(空)')==1
                        set(handles.BTWS_Set,'visible','off');
                        set(handles.QBXF_Set,'visible','off');
                        set(handles.FIR_Set,'visible','off');
                        set(handles.EllipticFilter_Set,'visible','off');
                        set(handles.Kalman_Set,'visible','off');
                        set(handles.FilterStartButton,'visible','off');
                end
                end
            end
        end
    end
end


% --- Executes on selection change in BTWSFilterType.
function BTWSFilterType_Callback(hObject, eventdata, handles)
% hObject    handle to BTWSFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns BTWSFilterType contents textbtwsas cell array
%        contents{get(hObject,'Value')} returns selected item from BTWSFilterType

%% 窗口使能与除能
strtemp=get(handles.BTWSFilterType,'string');
num=get(handles.BTWSFilterType,'value');
str=char(strtemp(num));
if strcmp(str,'高通')==1
        set(handles.textBTWSwp2,'visible','off');
        set(handles.BTWS_wp2,'visible','off');
        set(handles.BTWSwp2Hz,'visible','off');
        set(handles.textBTWSws2,'visible','off');
        set(handles.BTWS_ws2,'visible','off');
        set(handles.BTWSws2Hz,'visible','off');
else if strcmp(str,'低通')==1
        set(handles.textBTWSwp2,'visible','off');
        set(handles.BTWS_wp2,'visible','off');
        set(handles.BTWSwp2Hz,'visible','off');
        set(handles.textBTWSws2,'visible','off');
        set(handles.BTWS_ws2,'visible','off');
        set(handles.BTWSws2Hz,'visible','off');
    else if strcmp(str,'带通')==1
                    set(handles.textBTWSwp2,'visible','on');
                    set(handles.BTWS_wp2,'visible','on');
                    set(handles.BTWSwp2Hz,'visible','on');
                    set(handles.textBTWSws2,'visible','on');
                    set(handles.BTWS_ws2,'visible','on');
                    set(handles.BTWSws2Hz,'visible','on');
        else if strcmp(str,'带阻')==1
                        set(handles.textBTWSwp2,'visible','on');
                        set(handles.BTWS_wp2,'visible','on');
                        set(handles.BTWSwp2Hz,'visible','on');
                        set(handles.textBTWSws2,'visible','on');
                        set(handles.BTWS_ws2,'visible','on');
                        set(handles.BTWSws2Hz,'visible','on');
            end
        end
    end
end


%% --- Executes during object creation, after setting all properties.
function BTWSFilterType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWSFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_Rp_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_Rp textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_Rp textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_Rp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_As_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_As textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_As textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_As_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_wp1_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_wp1 textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_wp1 textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_wp1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_ws1_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_ws1 textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_ws1 textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_ws1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_wp2_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_wp2 textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_wp2 textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_wp2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BTWS_ws2_Callback(hObject, eventdata, handles)
% hObject    handle to BTWS_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BTWS_ws2 textbtwsas text
%        str2double(get(hObject,'String')) returns contents of BTWS_ws2 textbtwsas a double


% --- Executes during object creation, after setting all properties.
function BTWS_ws2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BTWS_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in FilterStartButton.
function FilterStartButton_Callback(hObject, eventdata, handles)
% hObject    handle to FilterStartButton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global CurrentFilterType
global GlobalDigitalFilterb_z
global GlobalDigitalFiltera_z
str=get(handles.FilterStartButton,'string');
if strcmp(str,'滤波开始') == 1
    if CurrentFilterType == handles.BTWS_Set
       %% 滤波器系数计算
            str=get(handles.BTWS_Rp,'string');
            Rp=str2num(str);
            str=get(handles.BTWS_As,'string');
            As=str2num(str);
            str=get(handles.BTWS_wp1,'string');
            wp1=str2num(str);
            str=get(handles.BTWS_ws1,'string');
            ws1=str2num(str);
            str=get(handles.BTWS_wp2,'string');
            wp2=str2num(str);
            str=get(handles.BTWS_ws2,'string');
            ws2=str2num(str);
            str=get(handles.Fs,'string');
            Fs=str2num(str);
            str=get(handles.BTWSFilterType,'string');
            num=get(handles.BTWSFilterType,'value');
            BTWSFilterType=char(str(num));
            T=1/Fs;
            wp1=2*pi*wp1/Fs;wp2=2*pi*wp2/Fs;    %数字带阻滤波器通带数字频率
            ws1=2*pi*ws1/Fs;ws2=2*pi*ws2/Fs;       %数字带阻滤波器阻带数字频率
            %频率预畸变,很重要。
            OmegaP1=(2/T)*tan(wp1/2); 
            OmegaP2=(2/T)*tan(wp2/2);
            OmegaS1=(2/T)*tan(ws1/2);
            OmegaS2=(2/T)*tan(ws2/2);
            if strcmp(BTWSFilterType,'低通') == 1
                OmegaP=[OmegaP1];
                OmegaS=[OmegaS1];
                BTWSFilterType='Low';
            else if strcmp(BTWSFilterType,'高通') == 1
                        OmegaP=[OmegaP1];
                        OmegaS=[OmegaS1];
                        BTWSFilterType='high';
                else if strcmp(BTWSFilterType,'带通') == 1
                            OmegaP=[OmegaP1,OmegaP2];
                            OmegaS=[OmegaS1,OmegaS2];
                            BTWSFilterType='bandpass';
                    else if strcmp(BTWSFilterType,'带阻') == 1
                                OmegaP=[OmegaP1,OmegaP2];
                                OmegaS=[OmegaS1,OmegaS2];
                                BTWSFilterType='stop';
                        end
                end
                end 
            end
            [N_d,OmegaC]=buttord(OmegaP,OmegaS,Rp,As,'s'); 
            %求滤波器阶数和3dB截止频率
            [b,a]=butter(N_d,OmegaC,BTWSFilterType,'s');  %构建模拟带阻滤波器
            [GlobalDigitalFilterb_z,GlobalDigitalFiltera_z]=bilinear(b,a,Fs);  %求数字带阻滤波器系数，双线性变换法
            set(handles.ClassicFilterOrder,'string',num2str(N_d));
        %% 窗口除能
            set(handles.BTWSFilterType,'enable','off');
            set(handles.BTWS_Rp,'enable','off');
            set(handles.BTWS_As,'enable','off');
            set(handles.BTWS_wp1,'enable','off');
            set(handles.BTWS_ws1,'enable','off');
            set(handles.BTWS_wp2,'enable','off');
            set(handles.BTWS_ws2,'enable','off');
    end
    if CurrentFilterType == handles.QBXF_Set
        %% 滤波器系数计算
            str=get(handles.QBXF_Rp,'string');
            Rp=str2num(str);
            str=get(handles.QBXF_As,'string');
            As=str2num(str);
            str=get(handles.QBXF_wp1,'string');
            wp1=str2num(str);
            str=get(handles.QBXF_ws1,'string');
            ws1=str2num(str);
            str=get(handles.QBXF_wp2,'string');
            wp2=str2num(str);
            str=get(handles.QBXF_ws2,'string');
            ws2=str2num(str);
            str=get(handles.Fs,'string');
            Fs=str2num(str);
            str=get(handles.QBXFFilterType,'string');
            num=get(handles.QBXFFilterType,'value');
            QBXFFilterType=char(str(num));
            T=1/Fs;
            wp1=2*pi*wp1/Fs;ws1=2*pi*ws1/Fs;
            wp2=2*pi*wp2/Fs;ws2=2*pi*ws2/Fs;
            OmegaP1=2*Fs*tan(wp1/2); 
            OmegaP2=2*Fs*tan(wp2/2);
            OmegaS1=2*Fs*tan(ws1/2);
            OmegaS2=2*Fs*tan(ws2/2);
             if strcmp(QBXFFilterType,'低通') == 1
                OmegaP=[OmegaP1];
                OmegaS=[OmegaS1];
                QBXFFilterType='Low';
            else if strcmp(QBXFFilterType,'高通') == 1
                        OmegaP=[OmegaP1];
                        OmegaS=[OmegaS1];
                        QBXFFilterType='high';
                else if strcmp(QBXFFilterType,'带通') == 1
                            OmegaP=[OmegaP1,OmegaP2];
                            OmegaS=[OmegaS1,OmegaS2];
                            QBXFFilterType='bandpass';
                    else if strcmp(QBXFFilterType,'带阻') == 1
                                OmegaP=[OmegaP1,OmegaP2];
                                OmegaS=[OmegaS1,OmegaS2];
                                QBXFFilterType='stop';
                        end
                end
                end 
             end
            [N_d,OmegaC]=cheb2ord(OmegaP,OmegaS,Rp,As,'s'); 
            %求滤波器阶数和3dB截止频率
            [b,a]=cheby2(N_d,As,OmegaC,QBXFFilterType,'s');  %构建模拟带阻滤波器
            [GlobalDigitalFilterb_z,GlobalDigitalFiltera_z]=bilinear(b,a,Fs);
             set(handles.ClassicFilterOrder,'string',num2str(N_d));
        %% 窗口除能
            set(handles.QBXFFilterType,'enable','off');
            set(handles.QBXF_Rp,'enable','off');
            set(handles.QBXF_As,'enable','off');
            set(handles.QBXF_wp1,'enable','off');
            set(handles.QBXF_ws1,'enable','off');
            set(handles.QBXF_wp2,'enable','off');
            set(handles.QBXF_ws2,'enable','off');
    end
    if CurrentFilterType == handles.EllipticFilter_Set
        %% 滤波器系数计算
            str=get(handles.Ellipse_Rp,'string');
            Rp=str2num(str);
            str=get(handles.Ellipse_As,'string');
            As=str2num(str);
            str=get(handles.Ellipse_wp1,'string');
            wp1=str2num(str);
            str=get(handles.Ellipse_ws1,'string');
            ws1=str2num(str);
            str=get(handles.Ellipse_wp2,'string');
            wp2=str2num(str);
            str=get(handles.Ellipse_ws2,'string');
            ws2=str2num(str);
            str=get(handles.Fs,'string');
            Fs=str2num(str);
            str=get(handles.EllipticFilterType,'string');
            num=get(handles.EllipticFilterType,'value');
            EllipticFilterType=char(str(num));
            T=1/Fs;
            wp1=2*pi*wp1/Fs;ws1=2*pi*ws1/Fs;
            wp2=2*pi*wp2/Fs;ws2=2*pi*ws2/Fs;
            OmegaP1=2*Fs*tan(wp1/2); 
            OmegaP2=2*Fs*tan(wp2/2);
            OmegaS1=2*Fs*tan(ws1/2);
            OmegaS2=2*Fs*tan(ws2/2);
             if strcmp(EllipticFilterType,'低通') == 1
                OmegaP=[OmegaP1];
                OmegaS=[OmegaS1];
                EllipticFilterType='Low';
            else if strcmp(EllipticFilterType,'高通') == 1
                        OmegaP=[OmegaP1];
                        OmegaS=[OmegaS1];
                        EllipticFilterType='high';
                else if strcmp(EllipticFilterType,'带通') == 1
                            OmegaP=[OmegaP1,OmegaP2];
                            OmegaS=[OmegaS1,OmegaS2];
                            EllipticFilterType='bandpass';
                    else if strcmp(EllipticFilterType,'带阻') == 1
                                OmegaP=[OmegaP1,OmegaP2];
                                OmegaS=[OmegaS1,OmegaS2];
                                EllipticFilterType='stop';
                        end
                end
                end 
             end
            [N_d,OmegaC]=ellipord(OmegaP,OmegaS,Rp,As,'s'); 
            %求滤波器阶数和3dB截止频率
            [b,a]=ellip(N_d,Rp,As,OmegaC,EllipticFilterType,'s');  %构建模拟带阻滤波器
            [GlobalDigitalFilterb_z,GlobalDigitalFiltera_z]=bilinear(b,a,Fs);
             set(handles.ClassicFilterOrder,'string',num2str(N_d));
        %% 窗口除能
                    set(handles.EllipticFilterType,'enable','off');
                    set(handles.Ellipse_Rp,'enable','off');
                    set(handles.Ellipse_As,'enable','off');
                    set(handles.Ellipse_wp1,'enable','off');
                    set(handles.Ellipse_ws1,'enable','off');
                    set(handles.Ellipse_wp2,'enable','off');
                    set(handles.Ellipse_ws2,'enable','off');
    end
    %% 窗口除能
    set(handles.FilterStartButton,'string','滤波停止');
    set(handles.FilterSelect,'enable','off');
else if  strcmp(str,'滤波停止')==1
        switch CurrentFilterType
            case handles.BTWS_Set 
                    %% 窗口使能
                    set(handles.BTWSFilterType,'enable','on');
                    set(handles.BTWS_Rp,'enable','on');
                    set(handles.BTWS_As,'enable','on');
                    set(handles.BTWS_wp1,'enable','on');
                    set(handles.BTWS_ws1,'enable','on');
                    set(handles.BTWS_wp2,'enable','on');
                    set(handles.BTWS_ws2,'enable','on');
            case handles.QBXF_Set
                    set(handles.QBXFFilterType,'enable','on');
                    set(handles.QBXF_Rp,'enable','on');
                    set(handles.QBXF_As,'enable','on');
                    set(handles.QBXF_wp1,'enable','on');
                    set(handles.QBXF_ws1,'enable','on');
                    set(handles.QBXF_wp2,'enable','on');
                    set(handles.QBXF_ws2,'enable','on');
            case handles.EllipticFilter_Set
                    set(handles.EllipticFilterType,'enable','on');
                    set(handles.Ellipse_Rp,'enable','on');
                    set(handles.Ellipse_As,'enable','on');
                    set(handles.Ellipse_wp1,'enable','on');
                    set(handles.Ellipse_ws1,'enable','on');
                    set(handles.Ellipse_wp2,'enable','on');
                    set(handles.Ellipse_ws2,'enable','on');
        end
        %% 窗口使能
        set(handles.FilterStartButton,'string','滤波开始');
        set(handles.FilterSelect,'enable','on');
    end
end


% --- Executes on selection change in FFTchannel.
function FFTchannel_Callback(hObject, eventdata, handles)
% hObject    handle to FFTchannel (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns FFTchannel contents as cell array
%        contents{get(hObject,'Value')} returns selected item from FFTchannel


% --- Executes on selection change in DataType.
function DataType_Callback(hObject, eventdata, handles)
% hObject    handle to DataType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns DataType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from DataType


% --- Executes during object creation, after setting all properties.
function DataType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DataType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_ws2_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_ws2 as text
%        str2double(get(hObject,'String')) returns contents of QBXF_ws2 as a double


% --- Executes during object creation, after setting all properties.
function QBXF_ws2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_wp2_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_wp2 as text
%        str2double(get(hObject,'String')) returns contents of QBXF_wp2 as a double


% --- Executes during object creation, after setting all properties.
function QBXF_wp2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_ws1_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_ws1 as text
%        str2double(get(hObject,'String')) returns contents of QBXF_ws1 as a double


% --- Executes during object creation, after setting all properties.
function QBXF_ws1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_wp1_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_wp1 as text
%        str2double(get(hObject,'String')) returns contents of QBXF_wp1 as a double


% --- Executes during object creation, after setting all properties.
function QBXF_wp1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_As_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_As as text
%        str2double(get(hObject,'String')) returns contents of QBXF_As as a double


% --- Executes during object creation, after setting all properties.
function QBXF_As_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function QBXF_Rp_Callback(hObject, eventdata, handles)
% hObject    handle to QBXF_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of QBXF_Rp as text
%        str2double(get(hObject,'String')) returns contents of QBXF_Rp as a double


% --- Executes during object creation, after setting all properties.
function QBXF_Rp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXF_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in QBXFFilterType.
function QBXFFilterType_Callback(hObject, eventdata, handles)
% hObject    handle to QBXFFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns QBXFFilterType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from QBXFFilterType
strtemp=get(handles.QBXFFilterType,'string');
num=get(handles.QBXFFilterType,'value');
str=char(strtemp(num));
if strcmp(str,'高通')==1
        set(handles.textQBXFwp2,'visible','off');
        set(handles.QBXF_wp2,'visible','off');
        set(handles.QBXFwp2Hz,'visible','off');
        set(handles.textQBXFws2,'visible','off');
        set(handles.QBXF_ws2,'visible','off');
        set(handles.QBXFws2Hz,'visible','off');
else if strcmp(str,'低通')==1
        set(handles.textQBXFwp2,'visible','off');
        set(handles.QBXF_wp2,'visible','off');
        set(handles.QBXFwp2Hz,'visible','off');
        set(handles.textQBXFws2,'visible','off');
        set(handles.QBXF_ws2,'visible','off');
        set(handles.QBXFws2Hz,'visible','off');
    else if strcmp(str,'带通')==1
                    set(handles.textQBXFwp2,'visible','on');
                    set(handles.QBXF_wp2,'visible','on');
                    set(handles.QBXFwp2Hz,'visible','on');
                    set(handles.textQBXFws2,'visible','on');
                    set(handles.QBXF_ws2,'visible','on');
                    set(handles.QBXFws2Hz,'visible','on');
        else if strcmp(str,'带阻')==1
                        set(handles.textQBXFwp2,'visible','on');
                        set(handles.QBXF_wp2,'visible','on');
                        set(handles.QBXFwp2Hz,'visible','on');
                        set(handles.textQBXFws2,'visible','on');
                        set(handles.QBXF_ws2,'visible','on');
                        set(handles.QBXFws2Hz,'visible','on');
            end
        end
    end
end


% --- Executes during object creation, after setting all properties.
function QBXFFilterType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to QBXFFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in DataNumberInit.
function DataNumberInit_Callback(hObject, eventdata, handles)
% hObject    handle to DataNumberInit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns DataNumberInit contents as cell array
%        contents{get(hObject,'Value')} returns selected item from DataNumberInit


% --- Executes during object creation, after setting all properties.
function DataNumberInit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to DataNumberInit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in EllipticFilterType.
function EllipticFilterType_Callback(hObject, eventdata, handles)
% hObject    handle to EllipticFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns EllipticFilterType contents as cell array
%        contents{get(hObject,'Value')} returns selected item from EllipticFilterType
strtemp=get(handles.EllipticFilterType,'string');
num=get(handles.EllipticFilterType,'value');
str=char(strtemp(num));
if strcmp(str,'高通')==1
        set(handles.textEllipsewp2,'visible','off');
        set(handles.Ellipse_wp2,'visible','off');
        set(handles.Ellipsewp2Hz,'visible','off');
        set(handles.textEllipsews2,'visible','off');
        set(handles.Ellipse_ws2,'visible','off');
        set(handles.Ellipsews2Hz,'visible','off');
else if strcmp(str,'低通')==1
        set(handles.textEllipsewp2,'visible','off');
        set(handles.Ellipse_wp2,'visible','off');
        set(handles.Ellipsewp2Hz,'visible','off');
        set(handles.textEllipsews2,'visible','off');
        set(handles.Ellipse_ws2,'visible','off');
        set(handles.Ellipsews2Hz,'visible','off');
    else if strcmp(str,'带通')==1
                    set(handles.textEllipsewp2,'visible','on');
                    set(handles.Ellipse_wp2,'visible','on');
                    set(handles.Ellipsewp2Hz,'visible','on');
                    set(handles.textEllipsews2,'visible','on');
                    set(handles.Ellipse_ws2,'visible','on');
                    set(handles.Ellipsews2Hz,'visible','on');
        else if strcmp(str,'带阻')==1
                        set(handles.textEllipsewp2,'visible','on');
                        set(handles.Ellipse_wp2,'visible','on');
                        set(handles.Ellipsewp2Hz,'visible','on');
                        set(handles.textEllipsews2,'visible','on');
                        set(handles.Ellipse_ws2,'visible','on');
                        set(handles.Ellipsews2Hz,'visible','on');
            end
        end
    end
end


% --- Executes during object creation, after setting all properties.
function EllipticFilterType_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EllipticFilterType (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_Rp_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_Rp as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_Rp as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_Rp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_Rp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_As_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_As as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_As as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_As_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_As (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_wp1_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_wp1 as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_wp1 as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_wp1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_wp1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_ws1_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_ws1 as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_ws1 as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_ws1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_ws1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_wp2_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_wp2 as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_wp2 as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_wp2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_wp2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Ellipse_ws2_Callback(hObject, eventdata, handles)
% hObject    handle to Ellipse_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Ellipse_ws2 as text
%        str2double(get(hObject,'String')) returns contents of Ellipse_ws2 as a double


% --- Executes during object creation, after setting all properties.
function Ellipse_ws2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Ellipse_ws2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes during object creation, after setting all properties.
function EllipticFilter_Set_CreateFcn(hObject, eventdata, handles)
% hObject    handle to EllipticFilter_Set (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
