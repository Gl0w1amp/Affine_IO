#include "serialslider.h"
#include <windows.h>
//#include <setupapi.h>
#include <stdio.h>
#include <conio.h>
#include <SetupAPI.h>

#define READ_BUF_SIZE 256
#define READ_TIMEOUT 500

#pragma comment(lib, "setupapi.lib")

char comPort[13] = {0};

const char* GetSerialPortByVidPid(const char* vid, const char* pid) {
    HDEVINFO deviceInfoSet;
    SP_DEVINFO_DATA deviceInfoData;
    DWORD i;
    char hardwareId[1024];
    static char portName[256];
	static char zero[10] = {0};

    // Get the device information set for all present devices
    deviceInfoSet = SetupDiGetClassDevs(NULL, "USB", NULL, DIGCF_PRESENT | DIGCF_ALLCLASSES);
    if (deviceInfoSet == INVALID_HANDLE_VALUE) {
		return zero;
    }

    deviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);

    // Enumerate through all devices in the set
    for (i = 0; SetupDiEnumDeviceInfo(deviceInfoSet, i, &deviceInfoData); i++) {
        // Get the hardware ID
        if (SetupDiGetDeviceRegistryProperty(deviceInfoSet, &deviceInfoData, SPDRP_HARDWAREID, NULL, (PBYTE)hardwareId, sizeof(hardwareId), NULL)) {
            // Check if the hardware ID contains the VID and PID
            if (strstr(hardwareId, vid) && strstr(hardwareId, pid)) {
                // Get the port name
                HKEY hDeviceKey = SetupDiOpenDevRegKey(deviceInfoSet, &deviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
                if (hDeviceKey != INVALID_HANDLE_VALUE) {
                    DWORD portNameSize = sizeof(portName);
                    if (RegQueryValueEx(hDeviceKey, "PortName", NULL, NULL, (LPBYTE)portName, &portNameSize) == ERROR_SUCCESS) {
                        RegCloseKey(hDeviceKey);
                        SetupDiDestroyDeviceInfoList(deviceInfoSet);
                        return portName;
                    }
                    RegCloseKey(hDeviceKey);
                }
            }
        }
    }

    SetupDiDestroyDeviceInfoList(deviceInfoSet);
    return zero;
}

// Global state
HANDLE hPort; // 串口句柄
DCB dcb; // 串口参数结构体
COMMTIMEOUTS timeouts; // 串口超时结构体
OVERLAPPED ovWrite;
BOOL fWaitingOnRead = FALSE, fWaitingOnWrite = FALSE;
slider_packet_t request;
BOOL Serial_Status;//串口状态（是否成功打开）

// Windows Serial helpers
BOOL open_port()
{
    // 打开串口
    hPort = CreateFile(comPort, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hPort == INVALID_HANDLE_VALUE)
    {
        //printf("can't open %s!\n", comPort);
        return FALSE;
    }

    // 获取串口参数
    GetCommState(hPort, &dcb);
    // 设置串口参数
    dcb.BaudRate = 115200; // 设置波特率
    dcb.ByteSize = 8; // 设置数据位为8
    dcb.Parity = NOPARITY; // 设置无奇偶校验
    dcb.StopBits = ONESTOPBIT; // 设置停止位为1
	dcb.fRtsControl = RTS_CONTROL_DISABLE;
	dcb.fOutxCtsFlow = FALSE;
    SetCommState(hPort, &dcb);

    // 获取串口超时
    GetCommTimeouts(hPort, &timeouts);

    // 设置串口超时
    
	timeouts.ReadIntervalTimeout = 1; // 设置读取间隔超时为1毫秒
    timeouts.ReadTotalTimeoutConstant = 5; // 设置读取总超时常量为5毫秒
    timeouts.ReadTotalTimeoutMultiplier = 1; // 设置读取总超时乘数为1毫秒
    timeouts.WriteTotalTimeoutConstant = 100; // 设置写入总超时常量为100毫秒
    timeouts.WriteTotalTimeoutMultiplier = 10; // 设置写入总超时乘数为10毫秒
    SetCommTimeouts(hPort, &timeouts);
    return TRUE;
}

void close_port(){
	CloseHandle(hPort);
	hPort = INVALID_HANDLE_VALUE;
}

// 检查串口是否打开
BOOL IsSerialPortOpen() {
    DWORD errors;
    COMSTAT status;

    ClearCommError(hPort, &errors, &status);
    if (errors > 0) {
        return FALSE;
    }

    return TRUE;
}

void package_init(slider_packet_t *request){
	for(uint8_t i = 0;i< BUFSIZE;i++){
		request->data[i] = 0;
	}
}

BOOL send_data(int length,uint8_t *send_buffer)
{
    DWORD bytes_written; // 写入的字节数
	// while (fWaitingOnWrite) {
    //     // Already waiting for a write to complete.
    // }
    // // Write data to the port asynchronously.
    // fWaitingOnWrite = TRUE;
    // 写入数据
    if (!WriteFile(hPort, send_buffer, length, &bytes_written, NULL))
    {
        // printf("can't write data to");
		// printf(comPort);
		// printf("/n");
        return FALSE;
    }
    // 返回成功
	//WriteFile(hPort, send_buffer, length, &bytes_written, &ovWrite);
    return TRUE;
}

static uint32_t millis() {
	return GetTickCount();
}

void sliderserial_writeresp(slider_packet_t *request) {
	uint8_t checksum = 0 - request->syn - request->cmd - request->size; 
	uint8_t length = request->size + 4;
	for (uint8_t i = 0;i<request->size;i++){
		checksum -= request->data[i+3];
		if((request->data[i+3] == 0xff) && (request->data[i+3] == 0xfd)){
			request->data[i+3] = 0xfe;
		}
	}
	request->data[request->size+3] = checksum;
	// if((request.checksum[0] == 0xff) && (request.checksum[0] == 0xfd)){
	// 	request.checksum[0] = 0xfd;
	// 	request.checksum[1] = 0xfe;
	// 	uint8_t length = request.size + 4;
	// }
	send_data(length,request->data);
}

BOOL serial_read1(uint8_t *result){
	long unsigned int recv_len;
	if (ReadFile(hPort,  result, 1, &recv_len, NULL) && (recv_len != 0)){
		return TRUE;
	}
	else{
		return false;
	}
	
}

uint8_t serial_read_cmd(slider_packet_t *reponse){
	uint8_t checksum = 0;
	uint8_t rep_size = 0;
	BOOL ESC = FALSE;
	uint8_t c;
	COMSTAT comStat;
	DWORD   dwErrors = 0;
	PurgeComm(hPort, PURGE_RXCLEAR);
	while(serial_read1(&c)){
		if(c == 0xff){
			package_init(reponse);
			rep_size = 0;
			reponse->syn = c;
			ESC = FALSE;
			checksum += 0xff;
			continue;
			}
		if(reponse->syn != 0xff){
			continue;
		}
		if(c == 0xfd){
			ESC = TRUE;
			continue;
		}
		if(ESC){
			c ++;
			ESC = FALSE;
		}
		if(reponse->cmd == 0){
			reponse->cmd = c;
			checksum += reponse->cmd;
			continue;
		}
		if(rep_size == 0){
			reponse->size = c;
			rep_size = 3;
			checksum += reponse->size;
			continue;
		}
		reponse->data[rep_size] = c;
		checksum += c;
		if ((rep_size == reponse->size + 3) || (rep_size >34) ){
			return reponse->cmd;
		}
		rep_size++;
	}
	if (!GetCommState(hPort, &dcb)) {
    // 串口已断开
	//printf("rff/n");
		return 0xff;
	}
	//printf("rfe/n");
	return 0xfe;
}

void slider_rst(){
	package_init(&request);
	request.syn = 0xff;
	request.cmd = SLIDER_CMD_RESET;
	request.size = 0;
	sliderserial_writeresp(&request);
	//Sleep(1);
}

void slider_start_scan(){
	package_init(&request);
	request.syn = 0xff;
	request.cmd = SLIDER_CMD_AUTO_SCAN_START;
	request.size = 0;
	sliderserial_writeresp(&request);
	Sleep(1);
}

void slider_stop_scan(){
	package_init(&request);
	request.syn = 0xff;
	request.cmd = SLIDER_CMD_AUTO_SCAN_STOP;
	request.size = 0;
	sliderserial_writeresp(&request);
	Sleep(1);
}

// void slider_stop_air_scan(){
// 	package_init(&request);
// 	request.syn = 0xff;
// 	request.cmd = SLIDER_CMD_AUTO_AIR_STOP;
// 	request.size = 0;
// 	sliderserial_writeresp(&request);
// 	//Sleep(1);
// }

void slider_send_leds(const uint8_t *rgb){
	package_init(&request);
	request.syn = 0xff;
	request.cmd = SLIDER_CMD_SET_LED;
	request.size = 96;
	memcpy(request.leds, rgb, 96);
	sliderserial_writeresp(&request);
	//Sleep(3);
}
