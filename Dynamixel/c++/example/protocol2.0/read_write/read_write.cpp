#ifdef __linux__
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif


#include <stdlib.h>
#include <stdio.h>
#include "dynamixel_sdk.h"                                  // Uses Dynamixel SDK library

//六軸センサ用
#include "stdafx.h"
#include <windows.h>
#include<iostream> //入出力ライブラリ
#include<fstream> //iostreamのファイル入出力をサポート
#include "CfsUsb.h"

//六軸センサ用関数
typedef void (CALLBACK *FUNC_Initialize)();
typedef void (CALLBACK *FUNC_Finalize)();
typedef bool (CALLBACK *FUNC_PortOpen)(int);
typedef void (CALLBACK *FUNC_PortClose)(int);
typedef bool (CALLBACK *FUNC_SetSerialMode)(int, bool);
typedef bool (CALLBACK* FUNC_GetSerialData)(int, double *, char *);
typedef bool (CALLBACK* FUNC_GetLatestData)(int, double *, char *);
typedef bool (CALLBACK *FUNC_GetSensorLimit)(int, double *);
typedef bool (CALLBACK* FUNC_GetSensorInfo)(int, char *);

// Control table address
#define ADDR_XM_CURRENT_LIMIT          38                   // Control table address is different in Dynamixel model
#define ADDR_XM_TORQUE_ENABLE          64
#define ADDR_XM_GOAL_CURRENT           102
#define ADDR_XM_GOAL_POSITION          116
#define ADDR_XM_PRESENT_CURRENT        126
#define ADDR_XM_PRESENT_VELOCITY       128
#define ADDR_XM_PRESENT_POSITION       132
#define ANGLE_RANGE                    4095
#define CURRENT_RANGE                  1193
#define ANGLE_AMOUNT                   0.088


#define CONTRACTURE_HIGH                90
#define CONTRACTURE_LOW                 5

// Protocol version
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL_ID                          1                   // Dynamixel ID: 1
#define BAUDRATE                        3000000
#define DEVICENAME                      "COM1"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define ESC_ASCII_VALUE                 0x1b

int getch()
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
#elif defined(_WIN32) || defined(_WIN64)
  return _getch();
#endif
}

int kbhit(void)
{
#ifdef __linux__
  struct termios oldt, newt;
  int ch;
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }

  return 0;
#elif defined(_WIN32) || defined(_WIN64)
  return _kbhit();
#endif
}


void error_check(int dxl_comm_result, dynamixel::PacketHandler *packetHandler, uint8_t dxl_error) {
	if (dxl_comm_result != COMM_SUCCESS)
	{
		packetHandler->printTxRxResult(dxl_comm_result);
	}
	else if (dxl_error != 0)
	{
		packetHandler->printRxPacketError(dxl_error);
	}
}

float cal_degree(int PresentAngle) {
	
	return (PresentAngle * ANGLE_AMOUNT - 180);

}

int cal_angle(float PresentDegree) {

	return ((PresentDegree+180)/ANGLE_AMOUNT);
}


int main()
{

  /***********Dynamixel用の変数************/
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  int index = 0;
  int dxl_comm_result = COMM_TX_FAIL;             // Communication result

  uint16_t iActiveDampingCurrentLimit;
  int iActiveDampingVelocityError;
  int iActiveDampingPreviousVelocityError = 0;
  int iActiveDampingVelocity = 0;
  int iActiveDampingCurrentOut;
  int dxl_present_angle_data = 0;

  int GoalAngle = 0;
  int GoalPosition = 2110;
  int PreGoalPosition;
  int error;
  int16_t PresentCurrent;
  int PresentAngle;
  float PresentDegree;
  int PresentVelocity;
  int PreCurrent;
  int AngleStartPosition;



  //f= kx + Ca
  float ElasticityGain = 0.5;
  float DampingGain = 1.0;

  float fActiveDampingPGain = 0.8;                // Damping coefficient would be larger if this value is increased.
  float fActiveDampingDGain = 0.3;                // When this value is initialized to negative number, the mechanical damping coefficient on the system would be canceled to each other.

  uint8_t dxl_error = 0;                          // Dynamixel error
  int32_t dxl_present_position = 0;               // Present position
  /*****************************************************/

  /***********六軸センサ用の変数***********/
  HMODULE hDll;
  long cnt;
  int portNo = 2;
  char SerialNo[9];
  char Status;
  double Limit[6];
  double Data[6];
  double Fx, Fy, Fz, Mx, My, Mz;
  double init_Fx, init_Fy, init_Fz, init_Mx, init_My, init_Mz;
  FUNC_Initialize Initialize;
  FUNC_Finalize Finalize;
  FUNC_PortOpen PortOpen;
  FUNC_PortClose PortClose;
  FUNC_SetSerialMode SetSerialMode;
  FUNC_GetSerialData GetSerialData;
  FUNC_GetLatestData GetLatestData;
  FUNC_GetSensorLimit GetSensorLimit;
  FUNC_GetSensorInfo GetSensorInfo;
  /*****************************************************/
  //ファイル出力先
  std::ofstream ofs("limit07.csv");

  // ＤＬＬのロード
  hDll = LoadLibrary("CfsUsb.dll");
  if (hDll != NULL)
  {
	  // 関数アドレスの取得
	  Initialize = (FUNC_Initialize)GetProcAddress(hDll, "Initialize");		// ＤＬＬの初期化処理
	  Finalize = (FUNC_Finalize)GetProcAddress(hDll, "Finalize");			// ＤＬＬの終了処理
	  PortOpen = (FUNC_PortOpen)GetProcAddress(hDll, "PortOpen");			// ポートオープン
	  PortClose = (FUNC_PortClose)GetProcAddress(hDll, "PortClose");		// ポートクローズ
	  SetSerialMode = (FUNC_SetSerialMode)GetProcAddress(hDll, "SetSerialMode");	// データの連続読込の開始/停止
	  GetSerialData = (FUNC_GetSerialData)GetProcAddress(hDll, "GetSerialData");	// 連続データ読込み
	  GetLatestData = (FUNC_GetLatestData)GetProcAddress(hDll, "GetLatestData");	// 最新データ読込
	  GetSensorLimit = (FUNC_GetSensorLimit)GetProcAddress(hDll, "GetSensorLimit");	// センサ定格確認
	  GetSensorInfo = (FUNC_GetSensorInfo)GetProcAddress(hDll, "GetSensorInfo");	// シリアルNo取得
	  // ＤＬＬの初期化処理
	  Initialize();
  }
  else
  {
	  printf("DLLのロードに失敗しました。");
	  return 0;
  }
  // 六軸センサポートオープン
  // 初期値の取得
  if (PortOpen(portNo) == true)
  {
	  // センサ定格確認
	  if (GetSensorLimit(portNo, Limit) == false)
	  {
		  printf("センサ定格確認ができません。");
	  }
	  // シリアルNo確認
	  if (GetSensorInfo(portNo, SerialNo) == false)
	  {
		  printf("シリアルNoが取得できません。");
	  }
	  /****************************/
	  /* ハンドシェイクによる読込 */
	  /****************************/
	  // 最新データ読込
	  // ※センサからは定格を10000としてデータが出力されてくる
	  if (GetLatestData(portNo, Data, &Status) == true)
	  {
		  init_Fx = Limit[0] / 10000 * Data[0];								// Fxの値
		  init_Fy = Limit[1] / 10000 * Data[1];								// Fyの値
		  init_Fz = Limit[2] / 10000 * Data[2];								// Fzの値
		  init_Mx = Limit[3] / 10000 * Data[3];								// Mxの値
		  init_My = Limit[4] / 10000 * Data[4];								// Myの値
		  init_Mz = Limit[5] / 10000 * Data[5];								// Mzの値

		  printf("GetLastData\n");
		  printf("Fx:%.1f Fy:%.1f Fz:%.1f Mx:%.2f My:%.2f Mz:%.2f\n", init_Fx, init_Fy, init_Fz, init_Mx, init_My, init_Mz);
	  }
	  else
	  {
		  printf("最新データ取得に失敗しました。");
	  }
  }
 

  // Dynamixel Open port
  if (portHandler->openPort())
  {
    printf("Succeeded to open the port!\n");
  }
  else
  {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE))
  {
    printf("Succeeded to change the baudrate!\n");
  }
  else
  {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }

  // Enable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  error_check(dxl_comm_result, packetHandler, dxl_error);

  // Read Dynamixel Current Limit
  dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_XM_CURRENT_LIMIT, &iActiveDampingCurrentLimit, &dxl_error);
  error_check(dxl_comm_result, packetHandler, dxl_error);


  printf("[ID:%03d] iActiveDampingCurrentLimit:%d\n", DXL_ID, iActiveDampingCurrentLimit);
  //電流の設定
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT, 0, &dxl_error);
  error_check(dxl_comm_result, packetHandler, dxl_error);
  

  while(1)
  {
	//Angle Dynamixelから取ってきた値
	//degree 実際の角度

    //printf("Press any key to continue! (or press ESC to quit!)\n");
    if(kbhit() != 0)
    {
      break;
    }
	
	//六軸センサのデータ取得
	if (GetLatestData(portNo, Data, &Status) == true)
	{
		Fx = (Limit[0] / 10000 * Data[0]) - init_Fx;						// Fxの値
		Fy = (Limit[1] / 10000 * Data[1]) - init_Fy;						// Fyの値
		Fz = (Limit[2] / 10000 * Data[2]) - init_Fz;						// Fzの値
		Mx = (Limit[3] / 10000 * Data[3]) - init_Mx;						// Mxの値
		My = (Limit[4] / 10000 * Data[4]) - init_My;						// Myの値
		Mz = (Limit[5] / 10000 * Data[5]) - init_Mz;						// Mzの値
		//printf("Fx:%.1f Fy:%.1f Fz:%.1f Mx:%.2f My:%.2f Mz:%.2f \n", Fx, Fy, Fz, Mx, My, Mz);
		ofs << "Fx:," << Fx << ",Fy:," << Fy << ",Fz:," << Fz << ",Mx:," << Mx << ",My:," << My << ",Mz:," << Mz << ",角度," << PresentDegree << std::endl;
	}
	else {
		//printf("失敗\n");
	}

	//角度取得
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_POSITION, (uint32_t*)&PresentAngle, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);
	//実際の角度
	PresentDegree = cal_degree(PresentAngle);//　°/1メモリ
	//printf("[ID:%03d] 角度:%f:%d \n", DXL_ID, PresentAngle,PresentAngle);

	//電流取得
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_CURRENT, (uint16_t *)&PresentCurrent, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);

	//ノイズ消す
	if (PresentCurrent<10 && PresentCurrent > -10) {
		PresentCurrent = 0;
	}
	//printf("[ID:%03d] 電流:%d \n", DXL_ID, PresentCurrent);
	

	// 速度取得
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_VELOCITY, (uint32_t*)&PresentVelocity, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);


	//終末域の場合
	if (PresentDegree < 0) {
		//電流をMAXに設定
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT,200, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);

		GoalAngle = PresentCurrent / ElasticityGain; //x(移動角度)　=　f(外力) / k（弾性係数
		AngleStartPosition = (0+180) / 0.088 ;
		GoalPosition = AngleStartPosition - GoalAngle;
		
		error = GoalPosition - PresentAngle;
		if (error>50 || error < -50) {
			GoalPosition = PresentAngle;
		}
		printf("[ID:%03d] 角度:%d \n", DXL_ID, GoalAngle);
		//角度を設定
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_POSITION, (uint32_t)GoalPosition, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);
	}
	else {
		//終末域以外はトルク0に設定
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT, 0, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);
	}
  }
  

  //六軸センサ
  PortClose(portNo);
  // ＤＬＬの終了処理
  Finalize();
  // ＤＬＬの解放
  FreeLibrary(hDll);


  // Disable Dynamixel Torque
  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_XM_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS)
  error_check(dxl_comm_result, packetHandler, dxl_error);
  
  // Close port
  portHandler->closePort();
  ofs.close();
  return 0;
}
