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

//�Z���Z���T�p
#include "stdafx.h"
#include <windows.h>
#include<iostream> //���o�̓��C�u����
#include<fstream> //iostream�̃t�@�C�����o�͂��T�|�[�g
#include "CfsUsb.h"

//�Z���Z���T�p�֐�
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

  /***********Dynamixel�p�̕ϐ�************/
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

  /***********�Z���Z���T�p�̕ϐ�***********/
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
  //�t�@�C���o�͐�
  std::ofstream ofs("limit07.csv");

  // �c�k�k�̃��[�h
  hDll = LoadLibrary("CfsUsb.dll");
  if (hDll != NULL)
  {
	  // �֐��A�h���X�̎擾
	  Initialize = (FUNC_Initialize)GetProcAddress(hDll, "Initialize");		// �c�k�k�̏���������
	  Finalize = (FUNC_Finalize)GetProcAddress(hDll, "Finalize");			// �c�k�k�̏I������
	  PortOpen = (FUNC_PortOpen)GetProcAddress(hDll, "PortOpen");			// �|�[�g�I�[�v��
	  PortClose = (FUNC_PortClose)GetProcAddress(hDll, "PortClose");		// �|�[�g�N���[�Y
	  SetSerialMode = (FUNC_SetSerialMode)GetProcAddress(hDll, "SetSerialMode");	// �f�[�^�̘A���Ǎ��̊J�n/��~
	  GetSerialData = (FUNC_GetSerialData)GetProcAddress(hDll, "GetSerialData");	// �A���f�[�^�Ǎ���
	  GetLatestData = (FUNC_GetLatestData)GetProcAddress(hDll, "GetLatestData");	// �ŐV�f�[�^�Ǎ�
	  GetSensorLimit = (FUNC_GetSensorLimit)GetProcAddress(hDll, "GetSensorLimit");	// �Z���T��i�m�F
	  GetSensorInfo = (FUNC_GetSensorInfo)GetProcAddress(hDll, "GetSensorInfo");	// �V���A��No�擾
	  // �c�k�k�̏���������
	  Initialize();
  }
  else
  {
	  printf("DLL�̃��[�h�Ɏ��s���܂����B");
	  return 0;
  }
  // �Z���Z���T�|�[�g�I�[�v��
  // �����l�̎擾
  if (PortOpen(portNo) == true)
  {
	  // �Z���T��i�m�F
	  if (GetSensorLimit(portNo, Limit) == false)
	  {
		  printf("�Z���T��i�m�F���ł��܂���B");
	  }
	  // �V���A��No�m�F
	  if (GetSensorInfo(portNo, SerialNo) == false)
	  {
		  printf("�V���A��No���擾�ł��܂���B");
	  }
	  /****************************/
	  /* �n���h�V�F�C�N�ɂ��Ǎ� */
	  /****************************/
	  // �ŐV�f�[�^�Ǎ�
	  // ���Z���T����͒�i��10000�Ƃ��ăf�[�^���o�͂���Ă���
	  if (GetLatestData(portNo, Data, &Status) == true)
	  {
		  init_Fx = Limit[0] / 10000 * Data[0];								// Fx�̒l
		  init_Fy = Limit[1] / 10000 * Data[1];								// Fy�̒l
		  init_Fz = Limit[2] / 10000 * Data[2];								// Fz�̒l
		  init_Mx = Limit[3] / 10000 * Data[3];								// Mx�̒l
		  init_My = Limit[4] / 10000 * Data[4];								// My�̒l
		  init_Mz = Limit[5] / 10000 * Data[5];								// Mz�̒l

		  printf("GetLastData\n");
		  printf("Fx:%.1f Fy:%.1f Fz:%.1f Mx:%.2f My:%.2f Mz:%.2f\n", init_Fx, init_Fy, init_Fz, init_Mx, init_My, init_Mz);
	  }
	  else
	  {
		  printf("�ŐV�f�[�^�擾�Ɏ��s���܂����B");
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
  //�d���̐ݒ�
  dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT, 0, &dxl_error);
  error_check(dxl_comm_result, packetHandler, dxl_error);
  

  while(1)
  {
	//Angle Dynamixel�������Ă����l
	//degree ���ۂ̊p�x

    //printf("Press any key to continue! (or press ESC to quit!)\n");
    if(kbhit() != 0)
    {
      break;
    }
	
	//�Z���Z���T�̃f�[�^�擾
	if (GetLatestData(portNo, Data, &Status) == true)
	{
		Fx = (Limit[0] / 10000 * Data[0]) - init_Fx;						// Fx�̒l
		Fy = (Limit[1] / 10000 * Data[1]) - init_Fy;						// Fy�̒l
		Fz = (Limit[2] / 10000 * Data[2]) - init_Fz;						// Fz�̒l
		Mx = (Limit[3] / 10000 * Data[3]) - init_Mx;						// Mx�̒l
		My = (Limit[4] / 10000 * Data[4]) - init_My;						// My�̒l
		Mz = (Limit[5] / 10000 * Data[5]) - init_Mz;						// Mz�̒l
		//printf("Fx:%.1f Fy:%.1f Fz:%.1f Mx:%.2f My:%.2f Mz:%.2f \n", Fx, Fy, Fz, Mx, My, Mz);
		ofs << "Fx:," << Fx << ",Fy:," << Fy << ",Fz:," << Fz << ",Mx:," << Mx << ",My:," << My << ",Mz:," << Mz << ",�p�x," << PresentDegree << std::endl;
	}
	else {
		//printf("���s\n");
	}

	//�p�x�擾
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_POSITION, (uint32_t*)&PresentAngle, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);
	//���ۂ̊p�x
	PresentDegree = cal_degree(PresentAngle);//�@��/1������
	//printf("[ID:%03d] �p�x:%f:%d \n", DXL_ID, PresentAngle,PresentAngle);

	//�d���擾
	dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_CURRENT, (uint16_t *)&PresentCurrent, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);

	//�m�C�Y����
	if (PresentCurrent<10 && PresentCurrent > -10) {
		PresentCurrent = 0;
	}
	//printf("[ID:%03d] �d��:%d \n", DXL_ID, PresentCurrent);
	

	// ���x�擾
	dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_XM_PRESENT_VELOCITY, (uint32_t*)&PresentVelocity, &dxl_error);
	error_check(dxl_comm_result, packetHandler, dxl_error);


	//�I����̏ꍇ
	if (PresentDegree < 0) {
		//�d����MAX�ɐݒ�
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT,200, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);

		GoalAngle = PresentCurrent / ElasticityGain; //x(�ړ��p�x)�@=�@f(�O��) / k�i�e���W��
		AngleStartPosition = (0+180) / 0.088 ;
		GoalPosition = AngleStartPosition - GoalAngle;
		
		error = GoalPosition - PresentAngle;
		if (error>50 || error < -50) {
			GoalPosition = PresentAngle;
		}
		printf("[ID:%03d] �p�x:%d \n", DXL_ID, GoalAngle);
		//�p�x��ݒ�
		dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_POSITION, (uint32_t)GoalPosition, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);
	}
	else {
		//�I����ȊO�̓g���N0�ɐݒ�
		dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_XM_GOAL_CURRENT, 0, &dxl_error);
		error_check(dxl_comm_result, packetHandler, dxl_error);
	}
  }
  

  //�Z���Z���T
  PortClose(portNo);
  // �c�k�k�̏I������
  Finalize();
  // �c�k�k�̉��
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
