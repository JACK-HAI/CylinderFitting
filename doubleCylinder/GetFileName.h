#pragma once
#include <tchar.h>
#include <wnnc.h>
#include <vector>
#include <windows.h>
#include <io.h>

using namespace std;
void GetFileFmat(LPCTSTR path, vector <wstring > &filesPathVector, TCHAR *fmat);

//��þ���·���е�����ļ���name��������path
//�β�nameΪ�洢�ļ����������׵�ַ
void get_name_from_pathname(char * const ppathname, char* name);
//void THCAR2char(TCHAR* tchStr, char chRtn[MAX_PATH]);
void TcharToChar(const TCHAR * tchar, char * _char);